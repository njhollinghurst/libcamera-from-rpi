/* SPDX-License-Identifier: BSD-2-Clause */
/*
 * Copyright (C) 2022-2023, Raspberry Pi Ltd
 *
 * Autofocus control algorithm
 */

#include "newaf.h"

#include <cmath>
#include <iomanip>
#include <stdlib.h>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>

using namespace RPiController;
using namespace libcamera;

LOG_DEFINE_CATEGORY(RPiNewAf)

#define NAME "rpi.new.af"

/*
 * Default values for parameters. All may be overridden in the tuning file.
 * Many of these values are sensor- or module-dependent; the defaults here
 * assume IMX708 in a Raspberry Pi V3 camera with the standard lens.
 *
 * Here all focus values are in dioptres (1/m). They are converted to hardware
 * units when written to status.lensSetting or returned from setLensPosition().
 *
 * Gain and delay values are relative to the update rate, since much (not all)
 * of the delay is in the sensor and (for CDAF) ISP, not the lens mechanism;
 * but note that algorithms are updated at no more than 30 Hz.
 */

NewAf::RangeDependentParams::RangeDependentParams()
	: focusMin(0.0),
	  focusMax(12.0),
	  focusDefault(1.0)
{
}

NewAf::SpeedDependentParams::SpeedDependentParams()
	: stepCoarse(1.0),
	  stepFine(0.25),
	  contrastRatio(0.75),
	  pdafGain(-0.02),
	  pdafSquelch(0.125),
	  maxSlew(2.0),
	  pdafFrames(20),
	  dropoutFrames(6),
	  stepFrames(4)
{
}

NewAf::CfgParams::CfgParams()
	: confEpsilon(8),
	  confThresh(16),
	  confClip(512),
	  skipFrames(5),
	  map()
{
}

template<typename T>
static void readNumber(T &dest, const libcamera::YamlObject &params, char const *name)
{
	auto value = params[name].get<T>();
	if (value)
		dest = *value;
	else
		LOG(RPiNewAf, Warning) << "Missing parameter \"" << name << "\"";
}

void NewAf::RangeDependentParams::read(const libcamera::YamlObject &params)
{

	readNumber<double>(focusMin, params, "min");
	readNumber<double>(focusMax, params, "max");
	readNumber<double>(focusDefault, params, "default");
}

void NewAf::SpeedDependentParams::read(const libcamera::YamlObject &params)
{
	readNumber<double>(stepCoarse, params, "step_coarse");
	readNumber<double>(stepFine, params, "step_fine");
	readNumber<double>(contrastRatio, params, "contrast_ratio");
	readNumber<double>(pdafGain, params, "pdaf_gain");
	readNumber<double>(pdafSquelch, params, "pdaf_squelch");
	readNumber<double>(maxSlew, params, "max_slew");
	readNumber<uint32_t>(pdafFrames, params, "pdaf_frames");
	readNumber<uint32_t>(dropoutFrames, params, "dropout_frames");
	readNumber<uint32_t>(stepFrames, params, "step_frames");
}

int NewAf::CfgParams::read(const libcamera::YamlObject &params)
{
	if (params.contains("ranges")) {
		auto &rr = params["ranges"];

		if (rr.contains("normal"))
			ranges[AfRangeNormal].read(rr["normal"]);
		else
			LOG(RPiNewAf, Warning) << "Missing range \"normal\"";

		ranges[AfRangeMacro] = ranges[AfRangeNormal];
		if (rr.contains("macro"))
			ranges[AfRangeMacro].read(rr["macro"]);

		ranges[AfRangeFull].focusMin = std::min(ranges[AfRangeNormal].focusMin,
							ranges[AfRangeMacro].focusMin);
		ranges[AfRangeFull].focusMax = std::max(ranges[AfRangeNormal].focusMax,
							ranges[AfRangeMacro].focusMax);
		ranges[AfRangeFull].focusDefault = ranges[AfRangeNormal].focusDefault;
		if (rr.contains("full"))
			ranges[AfRangeFull].read(rr["full"]);
	} else
		LOG(RPiNewAf, Warning) << "No ranges defined";

	if (params.contains("speeds")) {
		auto &ss = params["speeds"];

		if (ss.contains("normal"))
			speeds[AfSpeedNormal].read(ss["normal"]);
		else
			LOG(RPiNewAf, Warning) << "Missing speed \"normal\"";

		speeds[AfSpeedFast] = speeds[AfSpeedNormal];
		if (ss.contains("fast"))
			speeds[AfSpeedFast].read(ss["fast"]);
	} else
		LOG(RPiNewAf, Warning) << "No speeds defined";

	readNumber<uint32_t>(confEpsilon, params, "conf_epsilon");
	readNumber<uint32_t>(confThresh, params, "conf_thresh");
	readNumber<uint32_t>(confClip, params, "conf_clip");
	readNumber<uint32_t>(skipFrames, params, "skip_frames");

	if (params.contains("map"))
		map = params["map"].get<ipa::Pwl>(ipa::Pwl{});
	else
		LOG(RPiNewAf, Warning) << "No map defined";

	return 0;
}

void NewAf::CfgParams::initialise()
{
	if (map.empty()) {
		/* Default mapping from dioptres to hardware setting */
		static constexpr double DefaultMapX0 = 0.0;
		static constexpr double DefaultMapY0 = 445.0;
		static constexpr double DefaultMapX1 = 15.0;
		static constexpr double DefaultMapY1 = 925.0;

		map.append(DefaultMapX0, DefaultMapY0);
		map.append(DefaultMapX1, DefaultMapY1);
	}
}

/* Af Algorithm class */

static constexpr unsigned MaxWindows = 10;

NewAf::NewAf(Controller *controller)
	: AfAlgorithm(controller),
	  cfg_(),
	  range_(AfRangeNormal),
	  speed_(AfSpeedNormal),
	  mode_(AfAlgorithm::AfModeManual),
	  pauseFlag_(false),
	  statsRegion_(0, 0, 0, 0),
	  windows_(),
	  useWindows_(false),
	  phaseWeights_(),
	  contrastWeights_(),
	  skipCount_(0),
	  pdafCount_(0),
	  scanState_(ScanState::Idle),
	  scanStep_(0.0),
	  fineLimit_(0.0),
	  scanBounce_(false),
	  irFlag_(false),
	  scanMaxIndex_(0),
	  initted_(false),
	  ftarget_(-1.0),
	  fsmooth_(-1.0),
	  framesSameSign_(0),
	  framesSinceLastChange_(65536),
	  prevPhase_(0.0),
	  prevContrast_(0.0),
	  goodContrast_(0.0),
	  scanData_(),
	  reportState_(AfState::Idle)
{
	/*
	 * Reserve space for data, to reduce memory fragmentation. It's too early
	 * to query the size of the PDAF (from camera) and Contrast (from ISP)
	 * statistics, but these are plausible upper bounds.
	 */
	phaseWeights_.w.reserve(16 * 12);
	contrastWeights_.w.reserve(getHardwareConfig().focusRegions.width *
				   getHardwareConfig().focusRegions.height);
	scanData_.reserve(32);
}

NewAf::~NewAf()
{
}

char const *NewAf::name() const
{
	return NAME;
}

int NewAf::read(const libcamera::YamlObject &params)
{
	return cfg_.read(params);
}

void NewAf::initialise()
{
	cfg_.initialise();
}

void NewAf::switchMode(CameraMode const &cameraMode, [[maybe_unused]] Metadata *metadata)
{
	(void)metadata;

	/* Assume that PDAF and Focus stats grids cover the visible area */
	statsRegion_.x = (int)cameraMode.cropX;
	statsRegion_.y = (int)cameraMode.cropY;
	statsRegion_.width = (unsigned)(cameraMode.width * cameraMode.scaleX);
	statsRegion_.height = (unsigned)(cameraMode.height * cameraMode.scaleY);
	LOG(RPiNewAf, Debug) << "switchMode: statsRegion: "
			  << statsRegion_.x << ','
			  << statsRegion_.y << ','
			  << statsRegion_.width << ','
			  << statsRegion_.height;
	invalidateWeights();

	if (scanState_ != ScanState::Idle) {
		/*
		 * If a scan was in progress, re-start it, as CDAF statistics
		 * may have changed. Though if the application is just about
		 * to take a still picture, this will not help...
		 */
		startScan();
	}
	skipCount_ = cfg_.skipFrames;
}

void NewAf::computeWeights(RegionWeights *wgts, unsigned rows, unsigned cols)
{
	wgts->rows = rows;
	wgts->cols = cols;
	wgts->sum = 0;
	wgts->w.resize(rows * cols);
	std::fill(wgts->w.begin(), wgts->w.end(), 0);

	if (rows > 0 && cols > 0 && useWindows_ &&
	    statsRegion_.height >= rows && statsRegion_.width >= cols) {
		/*
		 * Here we just merge all of the given windows, weighted by area.
		 * \todo Perhaps a better approach might be to find the phase in each
		 * window and choose either the closest or the highest-confidence one?
		 * Ensure weights sum to less than (1<<16). 46080 is a "round number"
		 * below 65536, for better rounding when window size is a simple
		 * fraction of image dimensions.
		 */
		const unsigned maxCellWeight = 46080u / (MaxWindows * rows * cols);
		const unsigned cellH = statsRegion_.height / rows;
		const unsigned cellW = statsRegion_.width / cols;
		const unsigned cellA = cellH * cellW;

		for (auto &w : windows_) {
			for (unsigned r = 0; r < rows; ++r) {
				int y0 = std::max(statsRegion_.y + (int)(cellH * r), w.y);
				int y1 = std::min(statsRegion_.y + (int)(cellH * (r + 1)),
						  w.y + (int)(w.height));
				if (y0 >= y1)
					continue;
				y1 -= y0;
				for (unsigned c = 0; c < cols; ++c) {
					int x0 = std::max(statsRegion_.x + (int)(cellW * c), w.x);
					int x1 = std::min(statsRegion_.x + (int)(cellW * (c + 1)),
							  w.x + (int)(w.width));
					if (x0 >= x1)
						continue;
					unsigned a = y1 * (x1 - x0);
					a = (maxCellWeight * a + cellA - 1) / cellA;
					wgts->w[r * cols + c] += a;
					wgts->sum += a;
				}
			}
		}
	}

	if (wgts->sum == 0) {
		/* Default AF window is the middle 1/2 width of the middle 1/3 height */
		for (unsigned r = rows / 3; r < rows - rows / 3; ++r) {
			for (unsigned c = cols / 4; c < cols - cols / 4; ++c) {
				wgts->w[r * cols + c] = 1;
				wgts->sum += 1;
			}
		}
	}
}

void NewAf::invalidateWeights()
{
	phaseWeights_.sum = 0;
	contrastWeights_.sum = 0;
}

bool NewAf::getPhase(PdafRegions const &regions, double &phase, double &conf)
{
	libcamera::Size size = regions.size();
	if (size.height != phaseWeights_.rows || size.width != phaseWeights_.cols ||
	    phaseWeights_.sum == 0) {
		LOG(RPiNewAf, Debug) << "Recompute Phase weights " << size.width << 'x' << size.height;
		computeWeights(&phaseWeights_, size.height, size.width);
	}

	uint32_t sumWc = 0;
	int64_t sumWcp = 0;
	for (unsigned i = 0; i < regions.numRegions(); ++i) {
		unsigned w = phaseWeights_.w[i];
		if (w) {
			const PdafData &data = regions.get(i).val;
			unsigned c = data.conf;
			if (c >= (cfg_.confThresh >> 1)) {
				if (c > cfg_.confClip)
					c = cfg_.confClip;
				c -= (cfg_.confThresh >> 2);
				sumWc += w * c;
				sumWcp += (int64_t)(w * c) * (int64_t)data.phase;
			}
		}
	}

	if (0 < phaseWeights_.sum && phaseWeights_.sum <= sumWc &&
	    sumWc >= cfg_.confThresh * phaseWeights_.sum) {
		phase = (double)sumWcp / (double)sumWc;
		conf = (double)sumWc / (double)phaseWeights_.sum;
		return true;
	} else {
		phase = 0.0;
		conf = 0.0;
		return false;
	}
}

double NewAf::getContrast(const FocusRegions &focusStats)
{
	libcamera::Size size = focusStats.size();
	if (size.height != contrastWeights_.rows ||
	    size.width != contrastWeights_.cols || contrastWeights_.sum == 0) {
		LOG(RPiNewAf, Debug) << "Recompute Contrast weights "
				  << size.width << 'x' << size.height;
		computeWeights(&contrastWeights_, size.height, size.width);
	}

	uint64_t sumWc = 0;
	for (unsigned i = 0; i < focusStats.numRegions(); ++i)
		sumWc += contrastWeights_.w[i] * focusStats.get(i).val;

	return (contrastWeights_.sum > 0) ? ((double)sumWc / (double)contrastWeights_.sum) : 0.0;
}

double NewAf::findPeak(unsigned i) const
{
	double f = scanData_[i].focus;

	if (scanData_.size() >= 3) {
		/*
		 * Given the sample with the highest contrast score and its two
		 * neighbours either side (or same side if at the end of a scan),
		 * solve for the best lens position by fitting a parabola.
		 * Adapted from awb.cpp: interpolateQaudaratic()
		 */

		if (i == 0)
			i++;
		else if (i + 1 >= scanData_.size())
			i--;

		double abx = scanData_[i - 1].focus    - scanData_[i].focus;
		double aby = scanData_[i - 1].contrast - scanData_[i].contrast;
		double cbx = scanData_[i + 1].focus    - scanData_[i].focus;
		double cby = scanData_[i + 1].contrast - scanData_[i].contrast;
		double denom = 2.0 * (aby * cbx - cby * abx);
		if (std::abs(denom) >= (1.0/64.0) && denom * abx > 0.0) {
			f = (aby * cbx * cbx - cby * abx * abx) / denom;
			f = std::clamp(f, std::min(abx, cbx), std::max(abx, cbx));
			f += scanData_[i].focus;
		} else LOG(RPiNewAf, Debug) << "FindPeak FAILED";
	}

	LOG(RPiNewAf, Debug) << "FindPeak: " << f;
	return f;
}

bool NewAf::doPDAF(double phase, double conf)
{
	double delta = cfg_.speeds[speed_].pdafGain * phase;
	double close = true;

	delta *= conf / (conf + cfg_.confEpsilon);
	if (std::abs(delta) < cfg_.speeds[speed_].pdafSquelch) {
		double a = delta / cfg_.speeds[speed_].pdafSquelch;
		phase *= a * a;
	}
	if (delta < -cfg_.speeds[speed_].maxSlew) {
		delta = -cfg_.speeds[speed_].maxSlew;
		close = false;
	} else if (delta > cfg_.speeds[speed_].maxSlew) {
		delta = cfg_.speeds[speed_].maxSlew;
		close = false;
	}
	updateLensPosition(fsmooth_ + delta, true);
	return close;
}

void NewAf::doAF(double contrast, double phase, double conf)
{
	if (fsmooth_ != ftarget_) {
		fsmooth_ = std::clamp(ftarget_,
				      fsmooth_ - cfg_.speeds[speed_].maxSlew,
				      fsmooth_ + cfg_.speeds[speed_].maxSlew);
		return;
	}

	if (conf > 0.0 && phase * prevPhase_ > 0.0)
		framesSameSign_++;
	else
		framesSameSign_ = 0;

	if (skipCount_ > 0) {
		skipCount_--;
		return;
	}

	if (scanState_ == ScanState::PDAF) {
		if (pdafCount_ > 0) {
			--pdafCount_;
			if (conf > 0.0 && doPDAF(phase, conf)) {
				framesSinceLastChange_ = 0;
			}
			else if (++framesSinceLastChange_ == cfg_.speeds[speed_].dropoutFrames)
				startScan(true);
			return;
		}
		framesSinceLastChange_ = 65536;
		goodContrast_ = contrast;
		scanState_ = ScanState::Idle;
	}

	if (scanState_ == ScanState::Settle) {
		reportState_ = (contrast >= cfg_.speeds[speed_].contrastRatio * goodContrast_) ?
			AfState::Focused : AfState::Idle;
		goodContrast_ = contrast;
		framesSinceLastChange_ = 65536;
		scanState_ = ScanState::Idle;
	}

	if (scanState_ == ScanState::Idle) {
		if (mode_ != AfModeContinuous)
			return;
		if (framesSameSign_ >= 3) { /* XXX magic number */
			reportState_ =
				doPDAF(phase, conf) ? AfState::Focused : AfState::Failed;
			framesSinceLastChange_ = 65536;
			goodContrast_ = contrast;
		}
		if (contrast < cfg_.speeds[speed_].contrastRatio * goodContrast_ ||
		    goodContrast_ < cfg_.speeds[speed_].contrastRatio * contrast) {
			framesSinceLastChange_ = 0;
			goodContrast_ = contrast;
		} else if (++framesSinceLastChange_ == cfg_.speeds[speed_].dropoutFrames) {
			skipCount_ = 2 * cfg_.speeds[speed_].stepFrames;
			startScan(true);
		}
		return;
	}

	if (scanState_ == ScanState::ChooseDir) {
		if (conf > 0.0 && cfg_.speeds[speed_].pdafGain * phase > 0.0)
			scanStep_ = cfg_.speeds[speed_].stepCoarse;
		else
			scanStep_  = -cfg_.speeds[speed_].stepCoarse;
		LOG(RPiNewAf, Debug) << "Choose Dir " << ((scanStep_ < 0.0) ? -1 : 1);
		scanState_ = ScanState::Coarse;
		scanBounce_ = true;
	}

	/* If we got here we are in the middle of a scan: record data */
	ScanRecord r{fsmooth_, contrast, phase, conf};
	if (scanData_.empty() || contrast > goodContrast_) {
		scanMaxIndex_ = scanData_.size();
		goodContrast_ = contrast;
	}
	scanData_.push_back(r);
	double npos = ftarget_ + scanStep_;

	/* Early termination by PDAF! */
	if (scanData_.size() >= 3 && conf > 0) {
		unsigned i = scanData_.size() - 3;
		if (scanData_[i].conf > 0 && scanData_[i + 1].conf > 0 &&
		    (scanData_[i + 1].phase - scanData_[i].phase) * cfg_.speeds[speed_].pdafGain < 0.0 &&
		    (scanData_[i + 2].phase - scanData_[i + 1].phase) * cfg_.speeds[speed_].pdafGain < 0.0) {
			unsigned j = i + 2;
			if (scanData_[i].phase * scanData_[i + 1].phase < 0.0)
				j = i + 1;
			else if (scanData_[i + 1].phase * scanData_[j].phase < 0.0)
				i++;
			double den = std::abs(scanData_[j].phase - scanData_[i].phase);
			if (den > 0.001) {
				double s = std::abs(scanData_[i].phase) / den;
				if (s >= -2.0 && s <= 3.0) {
					npos = scanData_[i].focus + s * (scanData_[j].focus - scanData_[i].focus);
					LOG(RPiNewAf, Debug) << "ETBP" << scanData_[i].phase << ',' << scanData_[j].phase << ": " << npos;
					updateLensPosition(npos);
					scanState_ = ScanState::Settle;
				}
				return;
			}
		}
	}

	/* Termination conditions */
	double limit = (scanState_ == ScanState::Fine) ? fineLimit_ :
		(scanStep_ < 0.0) ? cfg_.ranges[range_].focusMin : cfg_.ranges[range_].focusMax;
	if ((scanData_.size() >= 3 && contrast < cfg_.speeds[speed_].contrastRatio * goodContrast_) ||
	    scanStep_ * (ftarget_ - limit) >= 0.0) {
		if (scanBounce_ && scanData_[0].contrast >= cfg_.speeds[speed_].contrastRatio * goodContrast_) {
			scanBounce_ = false;
			scanStep_ = -scanStep_;
			npos = (scanData_.size() >= 3) ? scanData_[1].focus : ftarget_;
		} else {
			npos = findPeak(scanMaxIndex_);
			if (0 && scanState_ == ScanState::Coarse) { /* XXX experimentally, let's omit the Fine scan altogether! */
				if (ftarget_ < npos) {
					fineLimit_ = std::min(npos + 2.0 * cfg_.speeds[speed_].stepFine, cfg_.ranges[range_].focusMax);
					npos -= 2.0 * cfg_.speeds[speed_].stepFine;
					scanStep_ = cfg_.speeds[speed_].stepFine;
				} else {
					fineLimit_ = std::max(npos - 2.0 * cfg_.speeds[speed_].stepFine, cfg_.ranges[range_].focusMin);
					npos += 2.0 * cfg_.speeds[speed_].stepFine;
					scanStep_ = -cfg_.speeds[speed_].stepFine;
				}
				scanState_ = ScanState::Fine;
			} else {
				scanState_ = ScanState::Settle;
			}
		}
		scanData_.clear();
	}
	updateLensPosition(npos);
}

void NewAf::updateLensPosition(double dioptres, bool continuous)
{
	/* Clamp to the  mapping (in manual mode) or selected range (for AF) */
	if (mode_ == AfAlgorithm::AfModeManual)
		dioptres = cfg_.map.domain().clamp(dioptres);
	else
		dioptres = std::clamp(dioptres,
				      cfg_.ranges[range_].focusMin,
				      cfg_.ranges[range_].focusMax);

	if (!initted_) {
		skipCount_ = std::max(skipCount_, cfg_.skipFrames);
		fsmooth_ = dioptres;
		initted_  = true;

	} else if (dioptres != ftarget_ && !continuous) {
		skipCount_ = std::max(skipCount_, cfg_.speeds[speed_].stepFrames);
	}
	ftarget_ = dioptres;
	fsmooth_ = std::clamp(ftarget_,
			      fsmooth_ - cfg_.speeds[speed_].maxSlew,
			      fsmooth_ + cfg_.speeds[speed_].maxSlew);
}

void NewAf::startScan(bool force)
{
	if (force ||
	    cfg_.speeds[speed_].pdafGain == 0.0 ||
	    cfg_.speeds[speed_].pdafFrames == 0 ||
	    cfg_.speeds[speed_].dropoutFrames == 0) {
		scanBounce_ = false;
		if (!initted_ ||
		    fsmooth_ <= cfg_.ranges[range_].focusMin + cfg_.speeds[speed_].stepCoarse) {
			scanState_ = ScanState::Coarse;
			scanStep_  = cfg_.speeds[speed_].stepCoarse;
			updateLensPosition(cfg_.ranges[range_].focusMin);
		} else if (fsmooth_ >= cfg_.ranges[range_].focusMax - cfg_.speeds[speed_].stepCoarse) {
			scanState_ = ScanState::Coarse;
			scanStep_  = -cfg_.speeds[speed_].stepCoarse;
			updateLensPosition(cfg_.ranges[range_].focusMax);
		} else {
			/* Use PDAF to choose the initial direction of motion */
			scanState_ = ScanState::ChooseDir;
		}
		framesSinceLastChange_ = 65536;
	} else {
		pdafCount_ = cfg_.speeds[speed_].pdafFrames;
		framesSinceLastChange_ = 0;
		scanState_ = ScanState::PDAF;
	}
	goodContrast_ = prevContrast_;
	scanData_.clear();
}

void NewAf::stopScan()
{
	scanState_ = ScanState::Idle;
	reportState_ = AfState::Idle;
	scanData_.clear();
	goodContrast_ = prevContrast_;
	framesSinceLastChange_ = 65536;
}

/*
 * PDAF phase data are available in prepare(), but CDAF statistics are not
 * available until process(). We are gambling on the availability of PDAF.
 * To expedite feedback control using PDAF, issue the V4L2 lens control from
 * prepare(). Conversely, during scans, we must allow an extra frame delay
 * between steps, to retrieve CDAF statistics from the previous process()
 * so we can terminate the scan early without having to change our minds.
 */

void NewAf::prepare(Metadata *imageMetadata)
{
	if (initted_) {
		/* Get PDAF from the embedded metadata, and run AF algorithm core */
		PdafRegions regions;
		double phase = 0.0, conf = 0.0;
		double oldFt = ftarget_;
		double oldFs = fsmooth_;
		double oldGc = goodContrast_;
		ScanState oldSs = scanState_;
		unsigned int oldSc = skipCount_;
		if (!irFlag_ && imageMetadata->get("pdaf.regions", regions) == 0)
			getPhase(regions, phase, conf);
		doAF(prevContrast_, phase, conf);
		prevPhase_ = phase;
		LOG(RPiNewAf, Debug) << std::fixed << std::setprecision(2)
				     << static_cast<unsigned int>(mode_)<< ':'
				     << static_cast<unsigned int>(reportState_)
				     << " skp" << oldSc << "->" << skipCount_
				     << " sst" << static_cast<unsigned int>(oldSs)
				     << "->" << static_cast<unsigned int>(scanState_)
				     << " ft" << oldFt << "->" << ftarget_
				     << " fs" << oldFs << "->" << fsmooth_
				     << " gc" << oldGc << "->" << goodContrast_
				     << " cont=" << (int)prevContrast_
				     << " phase=" << (int)phase << " conf=" << (int)conf;
	}

	/* Report status and produce new lens setting */
	AfStatus status;
	if (pauseFlag_)
		status.pauseState = (scanState_ == ScanState::Idle) ? AfPauseState::Paused
								    : AfPauseState::Pausing;
	else
		status.pauseState = AfPauseState::Running;

	if (mode_ == AfModeManual)
		status.state = AfState::Idle;
	else if (scanState_ != ScanState::Idle)
		status.state = AfState::Scanning;
	else
		status.state = reportState_;
	status.lensSetting = initted_ ? std::optional<int>(cfg_.map.eval(fsmooth_))
				      : std::nullopt;
	imageMetadata->set("af.status", status);
}

static bool checkForIR(const RgbyRegions & ss)
{
	uint64_t total = 0, count = 0;
	for (auto const &r : ss) {
		uint64_t avg3_2 = (r.val.rSum + r.val.gSum + r.val.bSum) >> 1;
		if (2 * r.val.rSum >= avg3_2 && r.val.rSum <= avg3_2 &&
		    2 * r.val.gSum >= avg3_2 && r.val.gSum <= avg3_2 &&
		    2 * r.val.bSum >= avg3_2 && r.val.bSum <= avg3_2)
			count += r.counted;
		total += r.counted;
	}
	LOG(RPiNewAf, Debug) << count << " / " << total << ((count > (total >> 1)) ? " Y" : " N");
	return count > (total >> 1);
}

void NewAf::process(StatisticsPtr &stats, [[maybe_unused]] Metadata *imageMetadata)
{
	(void)imageMetadata;
	prevContrast_ = getContrast(stats->focusRegions);
	irFlag_ = checkForIR(stats->awbRegions);
}

/* Controls */

void NewAf::setRange(AfRange r)
{
	LOG(RPiNewAf, Debug) << "setRange: " << (unsigned)r;
	if (r < AfAlgorithm::AfRangeMax)
		range_ = r;
}

void NewAf::setSpeed(AfSpeed s)
{
	LOG(RPiNewAf, Debug) << "setSpeed: " << (unsigned)s;
	if (s < AfAlgorithm::AfSpeedMax) {
		speed_ = s;
	}
}

void NewAf::setMetering(bool mode)
{
	if (useWindows_ != mode) {
		useWindows_ = mode;
		invalidateWeights();
	}
}

void NewAf::setWindows(libcamera::Span<libcamera::Rectangle const> const &wins)
{
	windows_.clear();
	for (auto &w : wins) {
		LOG(RPiNewAf, Debug) << "Window: "
				  << w.x << ", "
				  << w.y << ", "
				  << w.width << ", "
				  << w.height;
		windows_.push_back(w);
		if (windows_.size() >= MaxWindows)
			break;
	}

	if (useWindows_)
		invalidateWeights();
}

bool NewAf::setLensPosition(double dioptres, int *hwpos)
{
	bool changed = false;

	if (mode_ == AfModeManual) {
		LOG(RPiNewAf, Debug) << "setLensPosition: " << dioptres;
		changed = (dioptres != fsmooth_);
		updateLensPosition(dioptres);
	}

	if (hwpos)
		*hwpos = cfg_.map.eval(fsmooth_);

	return changed;
}

std::optional<double> NewAf::getLensPosition() const
{
	/*
	 * \todo We ought to perform some precise timing here to determine
	 * the current lens position.
	 */
	return (initted_) ? std::optional<double>(fsmooth_) : std::nullopt;
}

void NewAf::cancelScan()
{
	LOG(RPiNewAf, Debug) << "cancelScan";
	if (mode_ == AfModeAuto)
		stopScan();
}

void NewAf::triggerScan()
{
	LOG(RPiNewAf, Debug) << "triggerScan";
	if (mode_ == AfModeAuto && scanState_ == ScanState::Idle)
		startScan();
}

void NewAf::setMode(AfAlgorithm::AfMode mode)
{
	LOG(RPiNewAf, Debug) << "setMode: " << (unsigned)mode;
	if (mode_ != mode) {
		mode_ = mode;
		pauseFlag_ = false;
		if (mode == AfModeContinuous)
			startScan();
		else if (mode != AfModeAuto || scanState_ != ScanState::Idle)
			stopScan();
	}
}

AfAlgorithm::AfMode NewAf::getMode() const
{
	return mode_;
}

void NewAf::pause(AfAlgorithm::AfPause pause)
{
	LOG(RPiNewAf, Debug) << "pause: " << (unsigned)pause;
	if (mode_ == AfModeContinuous) {
		if (pause == AfPauseResume && pauseFlag_) {
			pauseFlag_ = false;
		} else if (pause != AfPauseResume && !pauseFlag_) {
			pauseFlag_ = true;
			if (pause == AfPauseImmediate || scanState_ != ScanState::Idle)
				stopScan();
		}
	}
}

// Register algorithm with the system.
static Algorithm *create(Controller *controller)
{
	return (Algorithm *)new NewAf(controller);
}
static RegisterAlgorithm reg(NAME, &create);

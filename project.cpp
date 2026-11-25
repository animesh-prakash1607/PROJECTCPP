#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace DisasterCore {

class SensorFailureException : public std::exception {
    std::string msg_;
public:
    explicit SensorFailureException(const std::string& node)
        : msg_("SensorFailureException: node [" + node + "] is unresponsive or power depleted") {}
    const char* what() const noexcept override { return msg_.c_str(); }
};

class TelemetryChecksumMismatch : public std::exception {
    std::string msg_;
public:
    TelemetryChecksumMismatch(const std::string& node, std::uint32_t expected, std::uint32_t received) {
        std::ostringstream os;
        os << "TelemetryChecksumMismatch: node [" << node << "] expected 0x" << std::hex << expected
           << " but received 0x" << received;
        msg_ = os.str();
    }
    const char* what() const noexcept override { return msg_.c_str(); }
};

class ThresholdOverflowException : public std::exception {
    std::string msg_;
public:
    ThresholdOverflowException(const std::string& node, double value, double limit) {
        std::ostringstream os;
        os << std::fixed << std::setprecision(2) << "ThresholdOverflowException: node [" << node
           << "] reading " << value << " exceeds hard limit " << limit;
        msg_ = os.str();
    }
    const char* what() const noexcept override { return msg_.c_str(); }
};

class LogTransactionException : public std::exception {
    std::string msg_;
public:
    explicit LogTransactionException(const std::string& reason)
        : msg_("LogTransactionException: " + reason) {}
    const char* what() const noexcept override { return msg_.c_str(); }
};

namespace Sensors {

struct GeoCoord {
    double lat;
    double lon;
};

struct RainSlope {
    double rainRate;
    double slopeAngle;
};

struct SoilVibration {
    double saturation;
    double vibrationHz;
};

inline double clampNorm(double v, double lo, double hi) {
    return v <= lo ? 0.0 : (v >= hi ? 1.0 : (v - lo) / (hi - lo));
}

inline double computeHazardIndex(const RainSlope& p) {
    return 100.0 * (0.6 * clampNorm(p.rainRate, 0.0, 120.0) + 0.4 * clampNorm(p.slopeAngle, 0.0, 60.0));
}

inline double computeHazardIndex(const SoilVibration& p) {
    return 100.0 * (0.7 * clampNorm(p.saturation, 0.0, 100.0) + 0.3 * clampNorm(p.vibrationHz, 0.0, 50.0));
}

inline double computeHazardIndex(double riverLevelM, double rainRate, double soilSaturation) {
    return 100.0 * (0.5 * clampNorm(riverLevelM, 0.0, 12.0) + 0.3 * clampNorm(rainRate, 0.0, 120.0) +
                    0.2 * clampNorm(soilSaturation, 0.0, 100.0));
}

inline int classifyLevel(double hazard) {
    return hazard < 30.0 ? 0 : (hazard < 55.0 ? 1 : (hazard < 75.0 ? 2 : 3));
}

inline const char* levelName(int level) {
    static const char* names[] = {"NORMAL", "ADVISORY", "WATCH", "WARNING"};
    return names[level < 0 ? 0 : (level > 3 ? 3 : level)];
}

inline void applyOffset(GeoCoord& c, double dLat, double dLon) {
    c.lat += dLat;
    c.lon += dLon;
}

inline GeoCoord& accumulate(GeoCoord& total, const GeoCoord& delta) {
    total.lat += delta.lat;
    total.lon += delta.lon;
    return total;
}

inline double& accumulatorSlot(std::vector<double>& acc, std::size_t index) {
    return acc.at(index);
}

class TelemetryBuffer {
    double* data_;
    std::size_t cap_;
    std::size_t size_;
    std::size_t head_;
public:
    explicit TelemetryBuffer(std::size_t capacity = 8)
        : data_(nullptr), cap_(capacity ? capacity : 1), size_(0), head_(0) {
        data_ = new double[cap_]();
    }

    TelemetryBuffer(std::size_t capacity, double fill)
        : data_(nullptr), cap_(capacity ? capacity : 1), size_(0), head_(0) {
        data_ = new double[cap_];
        std::fill(data_, data_ + cap_, fill);
        size_ = cap_;
    }

    TelemetryBuffer(const TelemetryBuffer& other)
        : data_(new double[other.cap_]), cap_(other.cap_), size_(other.size_), head_(other.head_) {
        std::copy(other.data_, other.data_ + other.cap_, data_);
    }

    TelemetryBuffer& operator=(const TelemetryBuffer& other) {
        if (this != &other) {
            double* fresh = new double[other.cap_];
            std::copy(other.data_, other.data_ + other.cap_, fresh);
            delete[] data_;
            data_ = fresh;
            cap_ = other.cap_;
            size_ = other.size_;
            head_ = other.head_;
        }
        return *this;
    }

    ~TelemetryBuffer() { delete[] data_; }

    void push(double v) {
        if (size_ < cap_) {
            data_[(head_ + size_) % cap_] = v;
            ++size_;
        } else {
            data_[head_] = v;
            head_ = (head_ + 1) % cap_;
        }
    }

    std::size_t size() const { return size_; }
    std::size_t capacity() const { return cap_; }

    double& at(std::size_t i) {
        if (i >= size_) throw std::out_of_range("TelemetryBuffer index out of range");
        return data_[(head_ + i) % cap_];
    }

    const double& at(std::size_t i) const {
        if (i >= size_) throw std::out_of_range("TelemetryBuffer index out of range");
        return data_[(head_ + i) % cap_];
    }

    double average() const {
        if (size_ == 0) return 0.0;
        double sum = 0.0;
        for (std::size_t i = 0; i < size_; ++i) sum += at(i);
        return sum / static_cast<double>(size_);
    }

    double peak() const {
        double best = 0.0;
        for (std::size_t i = 0; i < size_; ++i) best = std::max(best, at(i));
        return best;
    }
};

class TelemetryAuditor;

class SensorStation {
    int id_;
    std::string name_;
    GeoCoord pos_;
    double rain_;
    double slope_;
    double soil_;
    double vib_;
    double hazard_;
    TelemetryBuffer history_;
    static int activeCount_;

    void recompute() {
        hazard_ = 0.5 * (computeHazardIndex(RainSlope{rain_, slope_}) +
                         computeHazardIndex(SoilVibration{soil_, vib_}));
        history_.push(hazard_);
    }

public:
    SensorStation()
        : id_(0), name_("unassigned"), pos_(GeoCoord{0.0, 0.0}), rain_(0.0), slope_(0.0), soil_(0.0),
          vib_(0.0), hazard_(0.0), history_(8) {
        ++activeCount_;
    }

    SensorStation(int id, const std::string& name, const GeoCoord& pos, double rain, double slope,
                  double soil, double vib)
        : id_(id), name_(name), pos_(pos), rain_(rain), slope_(slope), soil_(soil), vib_(vib),
          hazard_(0.0), history_(8) {
        ++activeCount_;
        recompute();
    }

    SensorStation(const SensorStation& other)
        : id_(other.id_), name_(other.name_), pos_(other.pos_), rain_(other.rain_), slope_(other.slope_),
          soil_(other.soil_), vib_(other.vib_), hazard_(other.hazard_), history_(other.history_) {
        ++activeCount_;
    }

    SensorStation& operator=(const SensorStation&) = default;

    ~SensorStation() { --activeCount_; }

    void update(double rain, double slope, double soil, double vib) {
        rain_ = rain;
        slope_ = slope;
        soil_ = soil;
        vib_ = vib;
        recompute();
    }

    int id() const { return id_; }
    const std::string& name() const { return name_; }
    double hazard() const { return hazard_; }
    int level() const { return classifyLevel(hazard_); }
    GeoCoord& locate() { return pos_; }
    const GeoCoord& locate() const { return pos_; }
    const TelemetryBuffer& history() const { return history_; }

    static int activeStations() { return activeCount_; }

    bool operator>(const SensorStation& other) const { return hazard_ > other.hazard_; }

    friend class TelemetryAuditor;
    friend std::istream& operator>>(std::istream& is, SensorStation& s);
    friend std::ostream& operator<<(std::ostream& os, const SensorStation& s);
};

int SensorStation::activeCount_ = 0;

inline std::ostream& operator<<(std::ostream& os, const SensorStation& s) {
    std::ostringstream line;
    line << s.id_ << ' ' << s.name_ << ' ' << std::fixed << std::setprecision(4) << s.pos_.lat << ' '
         << s.pos_.lon << std::setprecision(2) << ' ' << s.rain_ << ' ' << s.slope_ << ' ' << s.soil_ << ' '
         << s.vib_;
    return os << line.str();
}

inline std::istream& operator>>(std::istream& is, SensorStation& s) {
    int id;
    std::string name;
    GeoCoord pos;
    double rain, slope, soil, vib;
    if (is >> id >> name >> pos.lat >> pos.lon >> rain >> slope >> soil >> vib) {
        s.id_ = id;
        s.name_ = name;
        s.pos_ = pos;
        s.update(rain, slope, soil, vib);
    }
    return is;
}

class TelemetryAuditor {
public:
    void audit(const SensorStation& s, std::ostream& os) const {
        std::ostringstream line;
        line << std::fixed << std::setprecision(2) << "  [AUDIT] " << std::left << std::setw(8) << s.name_
             << " rain=" << s.rain_ << " slope=" << s.slope_ << " soil=" << s.soil_ << " vib=" << s.vib_
             << " hazard=" << s.hazard_ << " bufferPeak=" << s.history_.peak();
        os << line.str() << '\n';
    }

    double rawLoad(const SensorStation& s) const { return s.rain_ + s.soil_ + s.vib_; }
};

class RegionalZone {
    std::string name_;
    std::vector<SensorStation> stations_;
public:
    RegionalZone() : name_("unassigned-zone") {}
    explicit RegionalZone(const std::string& name) : name_(name) {}

    void rename(const std::string& name) { name_ = name; }
    void add(const SensorStation& s) { stations_.push_back(s); }
    const std::string& name() const { return name_; }
    std::size_t size() const { return stations_.size(); }
    const std::vector<SensorStation>& stations() const { return stations_; }

    double meanHazard() const {
        if (stations_.empty()) return 0.0;
        double sum = 0.0;
        for (std::size_t i = 0; i < stations_.size(); ++i) sum += stations_[i].hazard();
        return sum / static_cast<double>(stations_.size());
    }

    const SensorStation* hottest() const {
        const SensorStation* best = nullptr;
        for (std::size_t i = 0; i < stations_.size(); ++i)
            if (best == nullptr || stations_[i] > *best) best = &stations_[i];
        return best;
    }

    RegionalZone operator+(const SensorStation& s) const {
        RegionalZone z(*this);
        z.add(s);
        return z;
    }

    RegionalZone operator+(const RegionalZone& other) const {
        RegionalZone z(*this);
        z.name_ = name_ + "+" + other.name_;
        for (std::size_t i = 0; i < other.stations_.size(); ++i) z.add(other.stations_[i]);
        return z;
    }
};

inline RegionalZone operator+(const SensorStation& a, const SensorStation& b) {
    RegionalZone z("aggregate");
    z.add(a);
    z.add(b);
    return z;
}

class PowerSubsystem {
    double level_;
    double capacity_;
public:
    explicit PowerSubsystem(double level = 100.0, double capacity = 100.0)
        : level_(level), capacity_(capacity) {}

    bool drain(double amount) {
        level_ = std::max(0.0, level_ - amount);
        return level_ > 0.0;
    }

    void charge(double amount) { level_ = std::min(capacity_, level_ + amount); }
    double level() const { return level_; }
};

class DisasterTelemetryDevice {
    int deviceId_;
    std::string label_;
    static int liveDevices_;
public:
    DisasterTelemetryDevice(int id, const std::string& label) : deviceId_(id), label_(label) {
        ++liveDevices_;
    }

    virtual ~DisasterTelemetryDevice() { --liveDevices_; }

    int deviceId() const { return deviceId_; }
    const std::string& label() const { return label_; }
    static int liveDevices() { return liveDevices_; }

    virtual std::string describe() const {
        return "Device#" + std::to_string(deviceId_) + " " + label_;
    }
};

int DisasterTelemetryDevice::liveDevices_ = 0;

class BaseSensorNode : public virtual DisasterTelemetryDevice {
    PowerSubsystem power_;
    double last_;
    std::uint32_t lcg_;
    double spike_;
    bool hasSpike_;

protected:
    double acquire(double base, double amplitude, double drainCost) {
        if (power_.level() < 5.0) throw SensorFailureException(label());
        power_.drain(drainCost);
        lcg_ = lcg_ * 1664525u + 1013904223u;
        double unit = static_cast<double>((lcg_ >> 8) & 0xFFFFu) / 65535.0;
        double v = base + (unit * 2.0 - 1.0) * amplitude;
        if (hasSpike_) {
            v = spike_;
            hasSpike_ = false;
        }
        last_ = v;
        return v;
    }

    bool judge(double threshold, double hardLimit) const {
        if (last_ > hardLimit) throw ThresholdOverflowException(label(), last_, hardLimit);
        return last_ >= threshold;
    }

    PowerSubsystem& power() { return power_; }

public:
    BaseSensorNode(int id, const std::string& label, double battery, std::uint32_t seed)
        : DisasterTelemetryDevice(id, label), power_(battery, 100.0), last_(0.0), lcg_(seed), spike_(0.0),
          hasSpike_(false) {}

    virtual ~BaseSensorNode() {}

    virtual double readTelemetry() = 0;
    virtual bool evaluateThreshold() const = 0;
    virtual std::string kind() const = 0;

    double lastReading() const { return last_; }
    double batteryLevel() const { return power_.level(); }

    void injectSpike(double value) {
        spike_ = value;
        hasSpike_ = true;
    }
};

class SolarPoweredNode : public virtual DisasterTelemetryDevice, public BaseSensorNode {
    double panelWatts_;
public:
    SolarPoweredNode(int id, const std::string& label, double panelWatts, double battery, std::uint32_t seed)
        : DisasterTelemetryDevice(id, label), BaseSensorNode(id, label, battery, seed),
          panelWatts_(panelWatts) {}

    void sunrise(double hours) { power().charge(panelWatts_ * hours * 0.1); }
    double panelWatts() const { return panelWatts_; }
};

class WirelessLoRaTransceiver : public virtual DisasterTelemetryDevice {
    int channelMHz_;
    int packets_;
public:
    WirelessLoRaTransceiver(int id, const std::string& label, int channelMHz)
        : DisasterTelemetryDevice(id, label), channelMHz_(channelMHz), packets_(0) {}

    static std::uint32_t checksum(double value) {
        std::uint32_t bits = static_cast<std::uint32_t>(static_cast<long long>(value * 100.0));
        return (bits * 2654435761u) ^ 0xA5A5u;
    }

    std::uint32_t transmit(double value) {
        ++packets_;
        return checksum(value);
    }

    int packetsSent() const { return packets_; }
    int channel() const { return channelMHz_; }
};

class SoilMoistureProbe : public SolarPoweredNode {
public:
    SoilMoistureProbe(int id, const std::string& label, double battery, std::uint32_t seed)
        : DisasterTelemetryDevice(id, label), SolarPoweredNode(id, label, 20.0, battery, seed) {}

    double readTelemetry() override { return acquire(62.0, 10.0, 1.5); }
    bool evaluateThreshold() const override { return judge(70.0, 100.0); }
    std::string kind() const override { return "SoilMoisture(%)"; }
};

class SeismicGeophone : public SolarPoweredNode {
public:
    SeismicGeophone(int id, const std::string& label, double battery, std::uint32_t seed)
        : DisasterTelemetryDevice(id, label), SolarPoweredNode(id, label, 15.0, battery, seed) {}

    double readTelemetry() override { return acquire(18.0, 6.0, 2.0); }
    bool evaluateThreshold() const override { return judge(25.0, 80.0); }
    std::string kind() const override { return "SeismicGeophone(Hz)"; }
};

class AcousticTiltmeter : public SolarPoweredNode {
public:
    AcousticTiltmeter(int id, const std::string& label, double battery, std::uint32_t seed)
        : DisasterTelemetryDevice(id, label), SolarPoweredNode(id, label, 12.0, battery, seed) {}

    double readTelemetry() override { return acquire(3.0, 1.5, 1.0); }
    bool evaluateThreshold() const override { return judge(4.0, 15.0); }
    std::string kind() const override { return "AcousticTilt(deg)"; }
};

class RiverSonarGauge : public SolarPoweredNode {
public:
    RiverSonarGauge(int id, const std::string& label, double battery, std::uint32_t seed)
        : DisasterTelemetryDevice(id, label), SolarPoweredNode(id, label, 10.0, battery, seed) {}

    double readTelemetry() override { return acquire(6.5, 1.2, 3.0); }
    bool evaluateThreshold() const override { return judge(7.5, 14.0); }
    std::string kind() const override { return "RiverSonar(m)"; }
};

class AutonomousHybridProbe : public SolarPoweredNode, public WirelessLoRaTransceiver {
    bool corruptNext_;
public:
    AutonomousHybridProbe(int id, const std::string& label, std::uint32_t seed)
        : DisasterTelemetryDevice(id, label), SolarPoweredNode(id, label, 40.0, 100.0, seed),
          WirelessLoRaTransceiver(id, label, 868), corruptNext_(false) {}

    void corruptNextPacket() { corruptNext_ = true; }

    double readTelemetry() override {
        double v = acquire(55.0, 15.0, 2.0);
        std::uint32_t expected = checksum(v);
        std::uint32_t sent = transmit(v);
        if (corruptNext_) {
            sent ^= 0xFFu;
            corruptNext_ = false;
        }
        if (sent != expected) throw TelemetryChecksumMismatch(label(), expected, sent);
        return v;
    }

    bool evaluateThreshold() const override { return judge(65.0, 120.0); }
    std::string kind() const override { return "HybridProbe(idx)"; }

    std::string describe() const override {
        return DisasterTelemetryDevice::describe() + " [LoRa " + std::to_string(channel()) + "MHz]";
    }
};

}

namespace Alerts {

class EarlyWarningSiren {
    std::string region_;
    int level_;
    bool sounding_;
    static int globalLevel_;
    static int soundingCount_;
public:
    explicit EarlyWarningSiren(const std::string& region) : region_(region), level_(0), sounding_(false) {}

    EarlyWarningSiren(const EarlyWarningSiren&) = delete;
    EarlyWarningSiren& operator=(const EarlyWarningSiren&) = delete;

    ~EarlyWarningSiren() {
        if (sounding_) --soundingCount_;
    }

    void activate(int level) {
        if (level <= 0) return;
        if (!sounding_) {
            sounding_ = true;
            ++soundingCount_;
        }
        level_ = level;
        if (level > globalLevel_) globalLevel_ = level;
    }

    void silence() {
        if (sounding_) {
            sounding_ = false;
            --soundingCount_;
            level_ = 0;
        }
    }

    bool isSounding() const { return sounding_; }
    int level() const { return level_; }
    const std::string& region() const { return region_; }

    static int globalLevel() { return globalLevel_; }
    static int soundingSirens() { return soundingCount_; }
    static void resetGlobal() { globalLevel_ = 0; }
};

int EarlyWarningSiren::globalLevel_ = 0;
int EarlyWarningSiren::soundingCount_ = 0;

struct Col {
    int width;
};

inline Col col(int width) { return Col{width}; }

inline std::ostream& operator<<(std::ostream& os, const Col& c) {
    os << std::left << std::setw(c.width);
    return os;
}

inline std::ostream& alertRule(std::ostream& os) {
    return os << '+' << std::string(6, '-') << '+' << std::string(14, '-') << '+' << std::string(10, '-')
              << '+' << std::string(11, '-') << '+' << std::string(34, '-') << "+\n";
}

inline std::ostream& alertHeader(std::ostream& os) {
    return os << "| " << col(4) << "Rank" << " | " << col(12) << "Station" << " | " << col(8) << "Hazard"
              << " | " << col(9) << "Level" << " | " << col(32) << "Dispatch action" << " |\n";
}

struct AlertRow {
    int rank;
    std::string station;
    double hazard;
    int level;
    std::string action;
};

inline std::ostream& operator<<(std::ostream& os, const AlertRow& r) {
    std::ostringstream h;
    h << std::fixed << std::setprecision(2) << r.hazard;
    os << "| " << col(4) << r.rank << " | " << col(12) << r.station << " | " << col(8) << h.str() << " | "
       << col(9) << Sensors::levelName(r.level) << " | " << col(32) << r.action << " |\n";
    return os;
}

struct LogHeader {
    char magic[4];
    std::uint32_t count;
};

struct IncidentRecord {
    std::uint32_t incidentId;
    char station[16];
    double hazard;
    std::int32_t level;
    std::uint32_t checksum;
};

inline std::uint32_t recordChecksum(const IncidentRecord& r) {
    std::uint32_t h = 2166136261u;
    auto mix = [&h](const void* data, std::size_t n) {
        const unsigned char* p = static_cast<const unsigned char*>(data);
        for (std::size_t i = 0; i < n; ++i) {
            h ^= p[i];
            h *= 16777619u;
        }
    };
    mix(&r.incidentId, sizeof r.incidentId);
    mix(r.station, sizeof r.station);
    mix(&r.hazard, sizeof r.hazard);
    mix(&r.level, sizeof r.level);
    return h;
}

inline IncidentRecord makeRecord(std::uint32_t id, const std::string& station, double hazard, int level) {
    IncidentRecord r;
    std::memset(&r, 0, sizeof r);
    r.incidentId = id;
    std::strncpy(r.station, station.c_str(), sizeof r.station - 1);
    r.hazard = hazard;
    r.level = level;
    r.checksum = recordChecksum(r);
    return r;
}

class IncidentLogger {
    std::string path_;
    std::fstream file_;
    std::uint32_t committed_;
    std::uint32_t pending_;
    bool inTx_;

    static std::streamoff offsetOf(std::uint32_t index) {
        return static_cast<std::streamoff>(sizeof(LogHeader)) +
               static_cast<std::streamoff>(index) * static_cast<std::streamoff>(sizeof(IncidentRecord));
    }

    void writeHeader(std::uint32_t count) {
        LogHeader h;
        std::memcpy(h.magic, "ALF1", 4);
        h.count = count;
        file_.clear();
        file_.seekp(0, std::ios::beg);
        file_.write(reinterpret_cast<const char*>(&h), sizeof h);
        file_.flush();
        if (!file_) throw LogTransactionException("header write failed");
    }

public:
    explicit IncidentLogger(const std::string& path) : path_(path), committed_(0), pending_(0), inTx_(false) {
        {
            std::ifstream probe(path_, std::ios::binary);
            if (!probe) {
                std::ofstream create(path_, std::ios::binary);
            }
        }
        file_.open(path_, std::ios::in | std::ios::out | std::ios::binary);
        if (!file_) throw LogTransactionException("cannot open " + path_);
        file_.seekg(0, std::ios::end);
        if (static_cast<std::streamoff>(file_.tellg()) < static_cast<std::streamoff>(sizeof(LogHeader))) {
            writeHeader(0);
        } else {
            LogHeader h;
            file_.seekg(0, std::ios::beg);
            file_.read(reinterpret_cast<char*>(&h), sizeof h);
            if (!file_ || std::memcmp(h.magic, "ALF1", 4) != 0) throw LogTransactionException("bad log header");
            committed_ = h.count;
        }
    }

    ~IncidentLogger() {
        if (inTx_) rollback();
    }

    void begin() {
        if (inTx_) throw LogTransactionException("transaction already open");
        inTx_ = true;
        pending_ = 0;
        file_.clear();
        file_.seekp(offsetOf(committed_), std::ios::beg);
    }

    void append(const IncidentRecord& r) {
        if (!inTx_) throw LogTransactionException("append outside transaction");
        if (r.hazard < 0.0 || r.hazard > 100.0)
            throw ThresholdOverflowException(r.station, r.hazard, 100.0);
        file_.write(reinterpret_cast<const char*>(&r), sizeof r);
        if (!file_) throw LogTransactionException("record write failed");
        ++pending_;
    }

    void commit() {
        if (!inTx_) throw LogTransactionException("commit outside transaction");
        file_.flush();
        writeHeader(committed_ + pending_);
        committed_ += pending_;
        pending_ = 0;
        inTx_ = false;
    }

    void rollback() {
        if (!inTx_) return;
        file_.clear();
        file_.seekp(offsetOf(committed_), std::ios::beg);
        pending_ = 0;
        inTx_ = false;
    }

    std::uint32_t committedCount() const { return committed_; }

    std::vector<IncidentRecord> readAll() {
        std::vector<IncidentRecord> out;
        file_.flush();
        file_.clear();
        file_.seekg(offsetOf(0), std::ios::beg);
        for (std::uint32_t i = 0; i < committed_; ++i) {
            IncidentRecord r;
            file_.read(reinterpret_cast<char*>(&r), sizeof r);
            if (!file_) throw LogTransactionException("truncated record");
            if (r.checksum != recordChecksum(r))
                throw TelemetryChecksumMismatch(r.station, recordChecksum(r), r.checksum);
            out.push_back(r);
        }
        return out;
    }
};

}

struct EvacuationPriority {
    bool operator()(const Sensors::SensorStation& a, const Sensors::SensorStation& b) const { return a > b; }
};

class RouteDispatcher {
    std::ostream* os_;
    int rank_;
    std::map<std::string, int> perLevel_;
public:
    explicit RouteDispatcher(std::ostream& os) : os_(&os), rank_(0) {}

    void operator()(const Sensors::SensorStation& s) {
        ++rank_;
        int lvl = s.level();
        std::string action;
        if (lvl >= 3) action = "EVACUATE via ridge road R-" + std::to_string(rank_);
        else if (lvl == 2) action = "PREPARE evacuation, route R-" + std::to_string(rank_);
        else if (lvl == 1) action = "STANDBY rescue crews";
        else action = "ROUTINE monitoring";
        ++perLevel_[Sensors::levelName(lvl)];
        *os_ << Alerts::AlertRow{rank_, s.name(), s.hazard(), lvl, action};
    }

    int dispatched() const { return rank_; }
    const std::map<std::string, int>& summary() const { return perLevel_; }
};

}

static void banner(const std::string& title) {
    std::cout << "\n==== " << title << " ====\n";
}

int main() {
    namespace S = DisasterCore::Sensors;
    namespace A = DisasterCore::Alerts;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Autonomous Landslide & Flash-Flood Early Warning Grid (ALF-EWG)\n";

    banner("UNIT I: namespaces, inline functions, overloading, call/return by reference, friend");
    S::RainSlope rs{85.0, 40.0};
    S::SoilVibration sv{78.0, 22.0};
    std::cout << "computeHazardIndex(rain+slope)         = " << S::computeHazardIndex(rs) << '\n';
    std::cout << "computeHazardIndex(saturation+vibration) = " << S::computeHazardIndex(sv) << '\n';
    std::cout << "computeHazardIndex(river+rain+soil)    = " << S::computeHazardIndex(9.4, 85.0, 78.0) << '\n';

    S::GeoCoord origin{30.7333, 79.0667};
    S::applyOffset(origin, 0.0125, -0.0040);
    S::GeoCoord drift{0.0, 0.0};
    S::accumulate(S::accumulate(drift, S::GeoCoord{0.0010, 0.0020}), S::GeoCoord{0.0005, -0.0010});
    std::cout << std::setprecision(4) << "transformed origin = (" << origin.lat << ", " << origin.lon
              << ")  accumulated drift = (" << drift.lat << ", " << drift.lon << ")\n"
              << std::setprecision(2);
    std::vector<double> acc(3, 0.0);
    S::accumulatorSlot(acc, 0) += 10.5;
    S::accumulatorSlot(acc, 0) += 4.25;
    S::accumulatorSlot(acc, 2) += 7.0;
    std::cout << "telemetry accumulator slots = [" << acc[0] << ", " << acc[1] << ", " << acc[2] << "]\n";

    banner("UNIT II: stations, static members, operators, deep copy, streams");
    std::istringstream feed(
        "1 KDR-01 30.7352 79.0669 95 48 88 31\n"
        "2 KDR-02 30.7410 79.0721 40 25 52 9\n"
        "3 JSM-01 30.5551 79.5654 110 55 92 40\n"
        "4 JSM-02 30.5602 79.5701 20 15 30 4\n"
        "5 JSM-03 30.5699 79.5588 70 38 70 18\n");
    std::vector<S::SensorStation> fleet;
    {
        S::SensorStation incoming;
        while (feed >> incoming) fleet.push_back(incoming);
        std::cout << "active station objects while loading (incl. temp) = " << S::SensorStation::activeStations()
                  << '\n';
    }
    std::cout << "active station objects after scope exit           = " << S::SensorStation::activeStations()
              << '\n';

    std::cout << "stations as streamed to console:\n";
    for (std::size_t i = 0; i < fleet.size(); ++i) std::cout << "  " << fleet[i] << '\n';

    {
        std::ofstream out("stations_snapshot.txt");
        for (std::size_t i = 0; i < fleet.size(); ++i) out << fleet[i] << '\n';
    }
    std::vector<S::SensorStation> restored;
    {
        std::ifstream in("stations_snapshot.txt");
        S::SensorStation tmp;
        while (in >> tmp) restored.push_back(tmp);
    }
    bool identical = restored.size() == fleet.size();
    for (std::size_t i = 0; identical && i < fleet.size(); ++i) {
        std::ostringstream a, b;
        a << fleet[i];
        b << restored[i];
        identical = (a.str() == b.str());
    }
    std::cout << "file round trip restored " << restored.size() << " stations, identical = "
              << (identical ? "yes" : "no") << '\n';
    std::remove("stations_snapshot.txt");

    std::cout << "JSM-01 > KDR-01 ? " << (fleet[2] > fleet[0] ? "yes" : "no") << '\n';
    std::cout << "KDR-02 > KDR-01 ? " << (fleet[1] > fleet[0] ? "yes" : "no") << '\n';

    S::SensorStation clone = fleet[0];
    clone.update(5.0, 5.0, 5.0, 1.0);
    std::cout << "deep copy check: original KDR-01 buffer avg = " << fleet[0].history().average()
              << ", clone buffer avg = " << clone.history().average() << " (sizes "
              << fleet[0].history().size() << " vs " << clone.history().size() << ")\n";

    S::TelemetryBuffer ring(4);
    for (int i = 1; i <= 6; ++i) ring.push(i * 10.0);
    S::TelemetryBuffer ringCopy = ring;
    ring.at(0) = 999.0;
    std::cout << "ring buffer (cap 4) after 6 pushes: oldest=" << ringCopy.at(0) << " newest="
              << ringCopy.at(3) << " peak=" << ringCopy.peak() << "; original modified oldest=" << ring.at(0)
              << '\n';
    S::TelemetryBuffer prefilled(3, 7.5);
    std::cout << "prefilled buffer size=" << prefilled.size() << " avg=" << prefilled.average() << '\n';

    std::map<std::string, S::RegionalZone> zones;
    {
        S::RegionalZone kedar = fleet[0] + fleet[1];
        kedar.rename("Kedarnath");
        zones["Kedarnath"] = kedar;
        S::RegionalZone joshi = (fleet[2] + fleet[3]) + fleet[4];
        joshi.rename("Joshimath");
        zones["Joshimath"] = joshi;
    }
    S::RegionalZone merged = zones["Kedarnath"] + zones["Joshimath"];
    std::cout << "regional zones (operator+ aggregation):\n";
    std::for_each(zones.begin(), zones.end(), [](const std::pair<const std::string, S::RegionalZone>& kv) {
        std::cout << "  " << std::left << std::setw(10) << kv.first << std::right << " stations="
                  << kv.second.size() << " meanHazard=" << kv.second.meanHazard()
                  << " hottest=" << kv.second.hottest()->name() << '\n';
    });
    std::cout << "  merged zone [" << merged.name() << "] stations=" << merged.size()
              << " meanHazard=" << merged.meanHazard() << '\n';

    S::TelemetryAuditor auditor;
    std::cout << "friend class TelemetryAuditor reading private metrics:\n";
    auditor.audit(fleet[2], std::cout);
    auditor.audit(fleet[3], std::cout);
    std::cout << "  raw load (rain+soil+vib) of KDR-01 = " << auditor.rawLoad(fleet[0]) << '\n';

    banner("UNIT II: sirens with static global alert level");
    std::vector<A::EarlyWarningSiren*> sirens;
    for (std::map<std::string, S::RegionalZone>::const_iterator it = zones.begin(); it != zones.end(); ++it) {
        A::EarlyWarningSiren* siren = new A::EarlyWarningSiren(it->first);
        siren->activate(it->second.hottest()->level());
        sirens.push_back(siren);
        std::cout << "  siren " << std::left << std::setw(10) << siren->region() << std::right
                  << " level=" << S::levelName(siren->level()) << " sounding=" << (siren->isSounding() ? "yes" : "no")
                  << '\n';
    }
    std::cout << "sounding sirens = " << A::EarlyWarningSiren::soundingSirens()
              << ", global alert level = " << S::levelName(A::EarlyWarningSiren::globalLevel()) << '\n';
    sirens[0]->silence();
    std::cout << "after silencing " << sirens[0]->region() << ": sounding sirens = "
              << A::EarlyWarningSiren::soundingSirens() << ", global level stays "
              << S::levelName(A::EarlyWarningSiren::globalLevel()) << '\n';
    for (std::size_t i = 0; i < sirens.size(); ++i) delete sirens[i];
    std::cout << "after siren cleanup: sounding sirens = " << A::EarlyWarningSiren::soundingSirens() << '\n';

    banner("UNIT III: virtual base classes, containership, runtime polymorphism");
    std::vector<S::BaseSensorNode*> nodes;
    nodes.push_back(new S::SoilMoistureProbe(101, "SOIL-A1", 100.0, 11u));
    S::SeismicGeophone* geophone = new S::SeismicGeophone(102, "GEO-B2", 100.0, 22u);
    nodes.push_back(geophone);
    nodes.push_back(new S::AcousticTiltmeter(103, "TILT-C3", 100.0, 33u));
    nodes.push_back(new S::RiverSonarGauge(104, "SONAR-D4", 10.0, 44u));
    S::AutonomousHybridProbe* hybrid = new S::AutonomousHybridProbe(105, "HYBRID-E5", 55u);
    nodes.push_back(hybrid);
    std::cout << "live devices after allocation = " << S::DisasterTelemetryDevice::liveDevices() << '\n';
    S::SolarPoweredNode& solarPath = *hybrid;
    S::WirelessLoRaTransceiver& loraPath = *hybrid;
    std::cout << "single shared DisasterTelemetryDevice in hybrid probe: id via SolarPoweredNode = "
              << solarPath.deviceId() << ", id via WirelessLoRaTransceiver = " << loraPath.deviceId() << '\n';

    for (int cycle = 1; cycle <= 4; ++cycle) {
        std::cout << "-- sweep cycle " << cycle << " --\n";
        if (cycle == 2) geophone->injectSpike(120.0);
        if (cycle == 3) hybrid->corruptNextPacket();
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            S::BaseSensorNode* n = nodes[i];
            std::cout << "  " << std::left << std::setw(40) << n->describe() << std::setw(20) << n->kind()
                      << std::right;
            try {
                double v = n->readTelemetry();
                bool alarm = n->evaluateThreshold();
                std::cout << " value=" << std::setw(7) << v << " battery=" << std::setw(6) << n->batteryLevel()
                          << (alarm ? "  ALARM" : "  ok") << '\n';
            } catch (const DisasterCore::SensorFailureException& e) {
                std::cout << "\n    caught -> " << e.what() << '\n';
            } catch (const DisasterCore::TelemetryChecksumMismatch& e) {
                std::cout << "\n    caught -> " << e.what() << '\n';
            } catch (const DisasterCore::ThresholdOverflowException& e) {
                std::cout << "\n    caught -> " << e.what() << '\n';
            }
        }
        for (std::size_t i = 0; i < nodes.size(); ++i) {
            S::SolarPoweredNode* solar = dynamic_cast<S::SolarPoweredNode*>(nodes[i]);
            if (solar != nullptr) solar->sunrise(1.0);
        }
    }
    std::cout << "LoRa packets sent by hybrid probe = " << hybrid->packetsSent() << '\n';
    for (std::size_t i = 0; i < nodes.size(); ++i) delete nodes[i];
    nodes.clear();
    std::cout << "live devices after virtual-destructor cleanup = " << S::DisasterTelemetryDevice::liveDevices()
              << '\n';

    banner("UNIT IV: STL, functors, manipulators, tabular alert dispatch");
    std::vector<S::SensorStation> ranked(fleet);
    std::sort(ranked.begin(), ranked.end(), DisasterCore::EvacuationPriority());
    std::cout << A::alertRule << A::alertHeader << A::alertRule;
    DisasterCore::RouteDispatcher dispatcher =
        std::for_each(ranked.begin(), ranked.end(), DisasterCore::RouteDispatcher(std::cout));
    std::cout << A::alertRule;
    std::cout << "dispatched " << dispatcher.dispatched() << " stations; level summary:";
    for (std::map<std::string, int>::const_iterator it = dispatcher.summary().begin();
         it != dispatcher.summary().end(); ++it)
        std::cout << ' ' << it->first << '=' << it->second;
    std::cout << '\n';

    banner("UNIT IV: binary incident log with transactional rollback");
    std::remove("incident_log.bin");
    {
        A::IncidentLogger logger("incident_log.bin");

        logger.begin();
        for (std::size_t i = 0; i < 3; ++i)
            logger.append(A::makeRecord(static_cast<std::uint32_t>(1001 + i), ranked[i].name(), ranked[i].hazard(),
                                        ranked[i].level()));
        logger.commit();
        std::cout << "transaction 1 committed, records on disk = " << logger.committedCount() << '\n';

        try {
            logger.begin();
            logger.append(A::makeRecord(2001, "JSM-02", 12.5, 0));
            logger.append(A::makeRecord(2002, "FAULTY-9", 140.0, 3));
            logger.commit();
        } catch (const std::exception& e) {
            std::cout << "transaction 2 failed -> " << e.what() << '\n';
            logger.rollback();
            std::cout << "rolled back, records on disk = " << logger.committedCount() << '\n';
        }

        logger.begin();
        logger.append(A::makeRecord(3001, "KDR-02", ranked[3].hazard(), ranked[3].level()));
        logger.commit();
        std::cout << "transaction 3 committed, records on disk = " << logger.committedCount() << '\n';

        std::vector<A::IncidentRecord> records = logger.readAll();
        std::cout << "records read back with checksum verification:\n";
        std::for_each(records.begin(), records.end(), [](const A::IncidentRecord& r) {
            std::cout << "  incident " << r.incidentId << "  " << std::left << std::setw(8) << r.station
                      << std::right << " hazard=" << std::setw(6) << r.hazard << " level="
                      << S::levelName(r.level) << '\n';
        });
    }
    std::remove("incident_log.bin");

    banner("SUMMARY");
    {
        S::SensorStation scoped(9, "TEMP-99", S::GeoCoord{30.0, 79.0}, 10.0, 10.0, 10.0, 1.0);
        std::cout << "live station objects (fleet, copies, zones, ranked list, scoped) = " << S::SensorStation::activeStations() << '\n';
    }
    std::cout << "live station objects after scoped object is destroyed = " << S::SensorStation::activeStations() << '\n';
    std::cout << "live devices at exit = " << S::DisasterTelemetryDevice::liveDevices() << '\n';
    return 0;
}
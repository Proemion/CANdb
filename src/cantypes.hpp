#ifndef CANTYPES_HPP_ML9DFK7A
#define CANTYPES_HPP_ML9DFK7A

#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include <math.h>
#include <boost/optional.hpp>
#include <boost/any.hpp>

enum class CANsignalType { Int, Float, String };
enum class CANsignalMuxType { NotMuxed = 0, Muxer, Muxed };
enum class CANsignalEndianness { BigEndianMotorola = 0, LittleEndianIntel = 1 };

struct CANsignal {
    std::string signal_name;
    std::uint8_t startBit;
    std::uint8_t signalSize;
    CANsignalEndianness endianness;
    bool valueSigned;
    std::double_t factor;
    std::double_t offset;
    std::double_t min;
    std::double_t max;
    std::string unit;
    std::vector<std::string> receivers;
    CANsignalMuxType muxType{ CANsignalMuxType::NotMuxed };
    // 16-bit for better debug printing via boost::optional
    boost::optional<std::uint16_t> muxNdx{ boost::none };
    boost::optional<boost::any> startValue { boost::none };
    boost::optional<std::string> comment{ boost::none };
    // 16-bit for better debug printing via boost::optional
    boost::optional<std::uint16_t> valueType{ boost::none };
    boost::optional<std::string> valueDescription{ boost::none };

    bool operator==(const CANsignal& rhs) const
    {
        return signal_name == rhs.signal_name;
    }
};

struct CANmessage {
    // Constructor required for vs2015 to be able to use initializer_list
    CANmessage(std::uint32_t _id, const std::string& _name = "",
        std::uint32_t _dlc = 0, const std::vector<std::string>& _ecus = { })
        : id(_id)
        , name(_name)
        , dlc(_dlc)
        , ecus(_ecus)
    {
    }

    std::uint32_t id;
    std::string name;
    std::uint32_t dlc;
    std::vector<std::string> ecus;
    boost::optional<std::uint32_t> cycleTime{ boost::none };
    boost::optional<std::string> comment{ boost::none };
};

namespace std {
template <> struct less<CANmessage> {
    bool operator()(const CANmessage& lhs, const CANmessage& rhs) const
    {
        return lhs.id < rhs.id;
    }
};
} // namespace std

using CANmessages_t = std::map<CANmessage, std::vector<CANsignal>>;

struct CANdb_t {
    struct ValTable {
        std::string identifier;

        struct ValTableEntry {
            std::uint32_t id;
            std::string ident;
        };
        std::vector<ValTableEntry> entries;
    };

    std::map<CANmessage, std::vector<CANsignal>> messages;
    std::string version;
    std::vector<std::string> nodes;
    std::vector<std::string> symbols;
    std::vector<std::string> ecus;
    std::vector<ValTable> val_tables;
    boost::optional<std::uint32_t> genMsgCycleTimeMin{ boost::none };
    boost::optional<std::uint32_t> genMsgCycleTimeMax{ boost::none };
    boost::optional<std::uint32_t> genMsgCycleTimeDefault{ boost::none };
    boost::optional<std::double_t> genSigStartValueMin{ boost::none };
    boost::optional<std::double_t> genSigStartValueMax{ boost::none };
    boost::optional<std::double_t> genSigStartValueDefault{ boost::none };
};

#endif /* end of include guard: CANTYPES_HPP_ML9DFK7A */

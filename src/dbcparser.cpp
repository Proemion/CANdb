#include "dbcparser.h"
#include "log.hpp"
#include <dbc_grammar.hpp>

#include <fstream>
#include <peglib.h>

#include <boost/algorithm/string/classification.hpp>
#include <boost/algorithm/string/erase.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/optional/optional_io.hpp>

namespace {
    const std::string ECU_MAGIC_NAME_NONE{ "Vector__XXX" };
}

template <typename T> auto take_first(T& container) -> typename T::value_type
{
    if (container.empty()) {
        throw std::runtime_error("container is empty");
    }
    auto v = container.front();
    container.pop_front();

    return v;
}

template <typename T> auto take_back(T& container) -> typename T::value_type
{
    if (container.empty()) {
        throw std::runtime_error("empty container");
    }
    const auto v = container.back();
    container.pop_back();

    return v;
}

template <typename T>
auto to_vector(const T& container) -> std::vector<typename T::value_type>
{
    std::vector<typename T::value_type> ret;
    std::transform(std::begin(container), std::end(container),
        std::back_inserter(ret), [](const auto& v) { return v; });
    return ret;
}

using namespace CANdb;
using strings = std::vector<std::string>;

std::string withLines(const std::string& dbcFile)
{
    strings split;

    auto withDots = dbcFile;
    boost::replace_all(withDots, " ", "$");
    boost::replace_all(withDots, "\t", "[t]");
    boost::split(split, withDots, boost::is_any_of("\n"));
    std::string buff;

    int counter{ 1 };
    for (const auto& line : split) {
        buff += std::to_string(counter++) + std::string{ ": " } + line
            + std::string{ "\n" };
    }

    return buff;
}

// Changes \r\n to \n
std::string dos2unix(const std::string& data)
{
    std::string noWindowsShit;
    boost::replace_all_copy(
        std::back_inserter(noWindowsShit), data, "\r\n", "\n");

    return noWindowsShit;
}

void setMessageComment(CANdb_t &canDb, uint32_t id, const std::string& comment)
{
    cdb_debug("Setting the comment for message {} to \"{}\"", id, comment);
    auto messageIt = std::find_if(canDb.messages.begin(), canDb.messages.end(),
        [id](const std::pair<const CANmessage, std::vector<CANsignal>>& entry) {
            return entry.first.id == id;
        });
    if (messageIt != canDb.messages.end()) {
        // This implementation is by no means optimal, because it requires
        // creating copies of the message/signal pair already in the map,
        // updating the message's comment, removing the old data and inserting
        // the new stuff. This is unfortunately necessary since std::map
        // requires keys to be immutable, so they cannot be updated directly.
        cdb_debug("Found the message that needs the new comment");
        CANmessage updatedMessage(messageIt->first);
        std::vector<CANsignal> existingSignals(messageIt->second);
        updatedMessage.comment = comment;
        canDb.messages.erase(messageIt->first);
        canDb.messages[updatedMessage] = existingSignals;
    }
}

void setSignalComment(CANdb_t &canDb, uint32_t id,
    const std::string& signalName, const std::string& comment)
{
    cdb_debug("Setting the comment for signal {}:{} to \"{}\"", id, signalName,
        comment);
    auto messageIt = std::find_if(canDb.messages.begin(), canDb.messages.end(),
        [id](const std::pair<const CANmessage, std::vector<CANsignal>>& entry)
            { return entry.first.id == id; });
    if (messageIt != canDb.messages.end()) {
        cdb_debug("Found the signal that needs the new comment");
        auto signalIt = std::find_if(messageIt->second.begin(),
            messageIt->second.end(),
            [signalName](const CANsignal& signal)
                { return signal.signal_name == signalName; });
        if (signalIt != messageIt->second.end()) {
            signalIt->comment = comment;
        }
    }
}

void setMessageCycleTime(CANdb_t &canDb, uint32_t id, std::uint32_t cycleTime)
{
    cdb_debug("Setting the cycle time for message {} to \"{}\"", id, cycleTime);
    auto messageIt = std::find_if(canDb.messages.begin(), canDb.messages.end(),
        [id](const std::pair<const CANmessage, std::vector<CANsignal>>& entry) {
            return entry.first.id == id;
        });
    if (messageIt != canDb.messages.end()) {
        // See notes in setMessageComment regarding this copy/remove/insert
        // implementation.
        cdb_debug("Found the message that needs the new cycle time");
        CANmessage updatedMessage(messageIt->first);
        std::vector<CANsignal> existingSignals(messageIt->second);
        updatedMessage.cycleTime = cycleTime;
        canDb.messages.erase(messageIt->first);
        canDb.messages[updatedMessage] = existingSignals;
    }
}

void setSignalStartValue(CANdb_t &canDb, uint32_t id,
    const std::string& signalName, const boost::any& value)
{
    cdb_debug("Setting the start value for signal {}:{}", id, signalName);
    auto messageIt = std::find_if(canDb.messages.begin(), canDb.messages.end(),
        [id](const std::pair<const CANmessage, std::vector<CANsignal>>& entry)
            { return entry.first.id == id; });
    if (messageIt != canDb.messages.end()) {
        cdb_debug("Found the signal that needs the new start value");
        auto signalIt = std::find_if(messageIt->second.begin(),
            messageIt->second.end(),
            [signalName](const CANsignal& signal)
                { return signal.signal_name == signalName; });
        if (signalIt != messageIt->second.end()) {
            signalIt->startValue = value;
        }
    }
}

void setSignalValueType(CANdb_t &canDb, uint32_t id,
    const std::string& signalName, uint8_t valueType)
{
    cdb_debug("Setting the value type for signal {}:{} to \"{}\"", id,
        signalName, static_cast<uint16_t>(valueType));
    auto messageIt = std::find_if(canDb.messages.begin(), canDb.messages.end(),
        [id](const std::pair<const CANmessage, std::vector<CANsignal>>& entry)
            { return entry.first.id == id; });
    if (messageIt != canDb.messages.end()) {
        cdb_debug("Found the signal that needs the new value type");
        auto signalIt = std::find_if(messageIt->second.begin(),
            messageIt->second.end(),
            [signalName](const CANsignal& signal)
                { return signal.signal_name == signalName; });
        if (signalIt != messageIt->second.end()) {
            signalIt->valueType = static_cast<std::uint16_t>(valueType);
        }
    }
}

void setSignalValueDescription(CANdb_t &canDb, uint32_t id,
    const std::string& signalName, const std::string& valueDescription)
{
    cdb_debug("Setting the value description for signal {}:{} to \"{}\"", id,
        signalName, valueDescription);
    auto messageIt = std::find_if(canDb.messages.begin(), canDb.messages.end(),
        [id](const std::pair<const CANmessage, std::vector<CANsignal>>& entry)
            { return entry.first.id == id; });
    if (messageIt != canDb.messages.end()) {
        cdb_debug("Found the signal that needs the new value description");
        auto signalIt = std::find_if(messageIt->second.begin(),
            messageIt->second.end(),
            [signalName](const CANsignal& signal)
                { return signal.signal_name == signalName; });
        if (signalIt != messageIt->second.end()) {
            signalIt->valueDescription = valueDescription;
        }
    }
}

bool DBCParser::parse(const std::string& data) noexcept
{
    auto noTabsData = dos2unix(data);

    peg::parser parser;

    parser.log = [](size_t l, size_t k, const std::string& s) {
        cdb_error("Parser log {}:{} {}", l, k, s);
    };

    if (!parser.load_grammar(dbc_grammar.c_str(), dbc_grammar.length())) {
        cdb_error("Unable to parse grammar");
        return false;
    }

    parser.enable_trace(
        [](const char* a, const char* k, long unsigned int,
            const peg::SemanticValues&, const peg::Context&,
            const peg::any&) { cdb_trace(" Parsing {} \"{}\"", a, k); });

    cdb_debug("DBC file  = \n{}", withLines(noTabsData));

    strings phrases;
    std::deque<std::string> idents, signs, ecu_tokens;
    std::deque<std::int64_t> numbers;

    CANsignalMuxType muxType = CANsignalMuxType::NotMuxed;
    int muxNdx = -1;

    using PhrasePair = std::pair<std::uint32_t, std::string>;
    std::vector<PhrasePair> phrasesPairs;

    parser["version"] = [this, &phrases](const peg::SemanticValues&) {
        if (phrases.empty()) {
            throw peg::parse_error("Version phrase not found");
        }
        can_db.version = take_back(phrases);
    };

    parser["phrase"] = [&phrases](const peg::SemanticValues& sv) {
        auto s = sv.token();
        boost::algorithm::erase_all(s, "\"");
        phrases.push_back(s);
    };

    parser["ns"] = [this, &idents](const peg::SemanticValues& sv) {
        can_db.symbols = to_vector(idents);
        cdb_debug("Found symbols {}", sv.token());
        idents.clear();
    };

    parser["TOKEN"] = [&idents](const peg::SemanticValues& sv) {
        auto s = sv.token();
        boost::algorithm::erase_all(s, "\n");
        idents.push_back(s);
    };

    parser["ECU_TOKEN"] = [&ecu_tokens](const peg::SemanticValues& sv) {
        auto s = sv.token();
        boost::algorithm::erase_all(s, "\n");
        ecu_tokens.push_back(s);
    };

    parser["bs"] = [](const peg::SemanticValues&) {
        // TODO: Implement me
        cdb_warn("TAG BS Not implemented");
    };

    parser["sign"] = [&signs](const peg::SemanticValues& sv) {
        cdb_trace("Found sign {}", sv.token());
        signs.push_back(sv.token());
    };

    parser["bu"] = [&idents, this](const peg::SemanticValues& sv) {
        can_db.ecus = to_vector(idents);
        cdb_debug("Found ecus [bu] {}", sv.token());
        idents.clear();
    };

    parser["bu_sl"] = [&idents, this](const peg::SemanticValues& sv) {
        can_db.ecus = to_vector(idents);
        cdb_debug("Found ecus [bu] {}", sv.token());
        idents.clear();
    };

    parser["number"] = [&numbers](const peg::SemanticValues& sv) {
        try {
            auto number = std::stoull(sv.token(), nullptr, 10);
            cdb_trace("Found number {}", number);
            numbers.push_back(number);
        } catch (const std::exception& ex) {
            cdb_error(
                "Unable to parse {} to a number from {}", sv.token(), sv.str());
        }
    };

    parser["number_phrase_pair"]
        = [&phrasesPairs, &numbers, &phrases](const peg::SemanticValues&) {
              phrasesPairs.push_back(
                  std::make_pair(take_back(numbers), take_back(phrases)));
          };

    parser["val_entry"] = [this, &phrasesPairs](const peg::SemanticValues&) {
        std::vector<CANdb_t::ValTable::ValTableEntry> tab;
        std::transform(phrasesPairs.begin(), phrasesPairs.end(),
            std::back_inserter(tab), [](const auto& p) {
                return CANdb_t::ValTable::ValTableEntry{ p.first, p.second };
            });
        can_db.val_tables.push_back(CANdb_t::ValTable{ "", tab });
        phrasesPairs.clear();
    };

    parser["muxer"] = [&muxType](const peg::SemanticValues&)
        { muxType = CANsignalMuxType::Muxer; };
    parser["mux_ndx"] = [&muxType, &muxNdx](const peg::SemanticValues& sv) {
        muxType = CANsignalMuxType::Muxed;
        muxNdx = std::stoi(sv.token());
    };

    std::vector<CANsignal> signals;
    parser["message"] = [this, &numbers, &signals, &idents, &muxType,
                            &muxNdx, &ecu_tokens](const peg::SemanticValues&) {
        cdb_debug(
            "Found a message {} signals = {}", idents.size(), signals.size());
        if (numbers.size() < 2 || idents.size() < 2) {
            return;
        }
        auto dlc = take_back(numbers);
        auto id = take_back(numbers);
        auto ecu = take_back(idents);
        auto name = take_back(idents);

        if (ecu == ECU_MAGIC_NAME_NONE) {
            ecu.clear();
        }

        const CANmessage msg{ static_cast<std::uint32_t>(id), name,
            static_cast<std::uint32_t>(dlc), ecu };
        cdb_debug("Found a message with id = {}", msg.id);
        can_db.messages[msg] = signals;
        signals.clear();
        numbers.clear();
        idents.clear();
        muxType = CANsignalMuxType::NotMuxed;
        muxNdx = -1;
        ecu_tokens.clear();
    };

    parser["signal"] = [&idents, &numbers, &phrases, &signals, &signs,
                           &ecu_tokens, &muxType,
                           &muxNdx](const peg::SemanticValues& sv) {
        cdb_debug("Found signal {}", sv.token());

        std::vector<std::string> receivers;
        std::copy_if (ecu_tokens.begin(), ecu_tokens.end(),
            std::back_inserter(receivers),
            [](const std::string& ecu){ return ecu != ECU_MAGIC_NAME_NONE; });
        auto unit = take_back(phrases);

        auto max = take_back(numbers);
        auto min = take_back(numbers);
        auto offset = take_back(numbers);
        auto factor = take_back(numbers);
        auto valueSigned = take_back(signs) == "-";

        CANsignalMuxType sigMuxType;
        boost::optional<std::uint16_t> sigMuxNdx;

        if (muxType == CANsignalMuxType::Muxed) {
            sigMuxType = CANsignalMuxType::Muxed;
            sigMuxNdx = static_cast<std::uint16_t>(muxNdx);
            cdb_debug("Muxed signal: sigMuxType {}, sigMuxNdx {}",
                static_cast<int>(sigMuxType), sigMuxNdx);
        } else if (muxType == CANsignalMuxType::Muxer) {
            sigMuxType = CANsignalMuxType::Muxer;
            cdb_debug("Signal muxer: sigMuxType {}, sigMuxNdx {}",
                static_cast<int>(sigMuxType), sigMuxNdx);
        } else {
            sigMuxType = CANsignalMuxType::NotMuxed;
            cdb_debug("Unmuxed signal: sigMuxType {}, sigMuxNdx {}",
                static_cast<int>(sigMuxType), sigMuxNdx);
        }

        auto endianness = take_back(numbers);
        auto signalSize = take_back(numbers);
        auto startBit = take_back(numbers);

        auto signal_name = take_back(idents);

        signals.push_back(
            CANsignal{ signal_name, static_cast<std::uint8_t>(startBit),
                static_cast<std::uint8_t>(signalSize),
                static_cast<CANsignalEndianness>(endianness), valueSigned,
                static_cast<float>(factor), static_cast<float>(offset),
                static_cast<float>(min), static_cast<float>(max), unit,
                receivers, sigMuxType, sigMuxNdx });

        muxType = CANsignalMuxType::NotMuxed;
        muxNdx = -1;
        ecu_tokens.clear();
    };

    parser["cm_bo"] = [&numbers, &phrases,
                           this] (const peg::SemanticValues& sv) {
        cdb_debug("Found cm_bo {}", sv.token());
        auto comment = take_back(phrases);
        auto id = static_cast<std::uint32_t>(take_back(numbers));
        setMessageComment(can_db, id, comment);
        cdb_debug("Message comment id={}, comment=\"{}\"", id, comment);
        phrases.clear();
        numbers.clear();
    };

    parser["cm_sg"] = [&numbers, &idents, &phrases,
                           this](const peg::SemanticValues& sv) {
        cdb_debug("Found cm_sg {}", sv.token());
        auto comment = take_back(phrases);
        auto name = take_back(idents);
        auto id = static_cast<std::uint32_t>(take_back(numbers));
        setSignalComment(can_db, id, name, comment);
        cdb_debug("Signal comment id={}, name={}, comment=\"{}\"", id, name,
            comment);
        phrases.clear();
        numbers.clear();
        idents.clear();
    };

    // TODO: HEX forms of BA_DEF_ BO_ and BA_DEF_ SG_

    parser["ba_def_bo_int_num"] = [&phrases, &numbers, this]
                                       (const peg::SemanticValues& sv) {
        cdb_debug("Found ba_def_bo_int_num {}", sv.token());
        auto max = static_cast<std::uint32_t>(take_back(numbers));
        auto min = static_cast<std::uint32_t>(take_back(numbers));
        auto attributeName = take_back(phrases);
        if (attributeName == "GenMsgCycleTime") {
            can_db.genMsgCycleTimeMin = min;
            can_db.genMsgCycleTimeMax = max;
            cdb_debug("Found GenMsgCycleTime range: {}-{}",
                can_db.genMsgCycleTimeMin,
                can_db.genMsgCycleTimeMax);
        }
        phrases.clear();
        numbers.clear();
    };

    parser["ba_def_sg_int_num"] = [&phrases, &numbers, this]
                                       (const peg::SemanticValues& sv) {
        cdb_debug("Found ba_def_sg_int_num {}", sv.token());
        auto max = static_cast<std::uint32_t>(take_back(numbers));
        auto min = static_cast<std::uint32_t>(take_back(numbers));
        auto attributeName = take_back(phrases);
        if (attributeName == "GenSigStartValue") {
            can_db.genSigStartValueMin = min;
            can_db.genSigStartValueMax = max;
            cdb_debug("Found GenSigStartValue range: {}-{}",
                can_db.genSigStartValueMin,
                can_db.genSigStartValueMax);
        }
        phrases.clear();
        numbers.clear();
    };

    parser["ba_def_def"] = [&phrases, &numbers, this]
                                (const peg::SemanticValues& sv) {
        cdb_debug("Found ba_def_def {}", sv.token());
        try {
            auto value = static_cast<std::uint32_t>(take_back(numbers));
            auto attributeName = take_back(phrases);
            if (attributeName == "GenMsgCycleTime") {
                can_db.genMsgCycleTimeDefault = value;
                cdb_debug("Found default GenMsgCycleTime: {}",
                    can_db.genMsgCycleTimeDefault);
            } else if (attributeName == "GenSigStartValue") {
                can_db.genSigStartValueDefault = value;
                cdb_debug("Found default GenSigStartValue: {}",
                    can_db.genSigStartValueDefault);
            }
        } catch (const std::exception &) {
            cdb_debug("Ignoring potentially unsupported BA_DEF_DEF_");
        }
        phrases.clear();
        numbers.clear();
    };

    parser["ba_bo"] = [&numbers, &phrases, this]
                           (const peg::SemanticValues& sv) {
        cdb_debug("Found ba_bo {}", sv.token());
        try {
            auto cycleTime = static_cast<std::uint32_t>(take_back(numbers));
            auto id = static_cast<std::uint32_t>(take_back(numbers));
            auto attributeName = take_back(phrases);
            if (attributeName == "GenMsgCycleTime") {
                setMessageCycleTime(can_db, id, cycleTime);
                cdb_debug("Found message cycle time id={}, time={}", id,
                    cycleTime);
            }
        } catch (const std::exception &) {
            cdb_debug("Ignoring potentially unsupported BA_ BO_");
        }
        phrases.clear();
        numbers.clear();
    };

    parser["ba_sg"] = [&numbers, &idents, &phrases, this]
                           (const peg::SemanticValues& sv) {
        cdb_debug("Found ba_sg {}", sv.token());
        boost::optional<std::uint32_t> idToSet;
        boost::optional<std::string> nameToSet;
        boost::optional<boost::any> valueToSet;
        if (numbers.size() == 2) {
            auto value = static_cast<std::uint32_t>(take_back(numbers));
            auto name = take_back(idents);
            auto id = static_cast<std::uint32_t>(take_back(numbers));
            auto attributeName = take_back(phrases);
            if (attributeName == "GenSigStartValue") {
                idToSet = id;
                nameToSet = name;
                valueToSet = value;
                cdb_debug("Found signal start value id={}, name={}, value={}",
                    id, name, value);
            }
        } else if (phrases.size() == 2) {
            auto value = take_back(phrases);
            auto name = take_back(idents);
            auto id = static_cast<std::uint32_t>(take_back(numbers));
            auto attributeName = take_back(phrases);
            if (attributeName == "GenSigStartValue") {
                idToSet = id;
                nameToSet = name;
                valueToSet = value;
                cdb_debug("Found signal start value id={}, name={}, " \
                    "value=\"{}\"", id, name, value);
            }
        } else {
            cdb_debug("Ignoring potentially unsupported BA_ SG_");
        }

        if (idToSet && nameToSet && valueToSet) {
            setSignalStartValue(can_db, idToSet.get(), nameToSet.get(),
                valueToSet.get());
        }

        phrases.clear();
        numbers.clear();
        idents.clear();
    };

    parser["sig_val"] = [&numbers, &idents, this]
                           (const peg::SemanticValues& sv) {
        cdb_debug("Found val_ {}", sv.token());
        auto typeId = static_cast<uint8_t>(take_back(numbers));
        auto name = take_back(idents);
        auto id = static_cast<uint32_t>(take_back(numbers));
        cdb_debug("Value type for signal {}:{}: \"{}\"", id, name,
            static_cast<uint16_t>(typeId));
        setSignalValueType(can_db, id, name, typeId);

        numbers.clear();
        idents.clear();
    };

    parser["vals"] = [&numbers, &idents, &phrases, this]
                           (const peg::SemanticValues& sv) {
        cdb_debug("Found val_ {}", sv.token());
        std::string valueDescription;
        while (phrases.size() > 0 && numbers.size() > 0) {
            if (!valueDescription.empty()) {
                valueDescription.insert(0, " ");
            }
            valueDescription.insert(0, " " + take_back(phrases));
            valueDescription.insert(0,
                std::to_string(static_cast<uint32_t>(take_back(numbers))));
        }
        auto name = take_back(idents);
        auto id = static_cast<std::uint32_t>(take_back(numbers));
        cdb_debug("Value description for signal {}:{}: \"{}\"", id, name,
            valueDescription);
        setSignalValueDescription(can_db, id, name, valueDescription);

        phrases.clear();
        numbers.clear();
        idents.clear();
    };

    return parser.parse(noTabsData.c_str());
}

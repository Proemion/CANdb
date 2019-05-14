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
        cdb_debug("Message comment id={}, comment={}", id, comment);
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
        cdb_debug("Signal comment id={}, name={}, comment={}", id, name,
            comment);
        phrases.clear();
        numbers.clear();
        idents.clear();
    };

    std::uint32_t genMsgCycleTimeMin{ 0 };
    std::uint32_t genMsgCycleTimeMax{ 0 };
    parser["ba_def_bo_int_num"] = [&phrases, &numbers, &genMsgCycleTimeMin,
                                       &genMsgCycleTimeMax]
                                       (const peg::SemanticValues& sv) {
        cdb_debug("Found ba_def_bo_int_num {}", sv.token());
        auto max = static_cast<std::uint32_t>(take_back(numbers));
        auto min = static_cast<std::uint32_t>(take_back(numbers));
        auto attributeName = take_back(phrases);
        if (attributeName == "GenMsgCycleTime") {
            genMsgCycleTimeMin = min;
            genMsgCycleTimeMax = max;
            cdb_debug("Found GenMsgCycleTime range: {}-{}",
                genMsgCycleTimeMin,
                genMsgCycleTimeMax);
        }
        phrases.clear();
        numbers.clear();
    };

    std::uint32_t genSigStartValueMin{ 0 };
    std::uint32_t genSigStartValueMax{ 0 };
    parser["ba_def_sg_int_num"] = [&phrases, &numbers, &genSigStartValueMin,
                                       &genSigStartValueMax]
                                       (const peg::SemanticValues& sv) {
        cdb_debug("Found ba_def_sg_int_num {}", sv.token());
        auto max = static_cast<std::uint32_t>(take_back(numbers));
        auto min = static_cast<std::uint32_t>(take_back(numbers));
        auto attributeName = take_back(phrases);
        if (attributeName == "GenSigStartValue") {
            genSigStartValueMin = min;
            genSigStartValueMax = max;
            cdb_debug("Found GenSigStartValue range: {}-{}",
                genSigStartValueMin,
                genSigStartValueMax);
        }
        phrases.clear();
        numbers.clear();
    };

    std::uint32_t genMsgCycleTimeDefault{ 0 };
    std::uint32_t genSigStartValueDefault{ 0 };
    parser["ba_def_def"] = [&phrases, &numbers,
                                &genMsgCycleTimeDefault,
                                &genSigStartValueDefault]
                                (const peg::SemanticValues& sv) {
        cdb_debug("Found ba_def_def {}", sv.token());
        try {
            auto value = static_cast<std::uint32_t>(take_back(numbers));
            auto attributeName = take_back(phrases);
            if (attributeName == "GenMsgCycleTime") {
                genMsgCycleTimeDefault = value;
                cdb_debug("Found default GenMsgCycleTime: {}",
                    genMsgCycleTimeDefault);
            } else if (attributeName == "GenSigStartValue") {
                genSigStartValueDefault = value;
                cdb_debug("Found default GenSigStartValue: {}",
                    genSigStartValueDefault);
            }
        } catch (const std::exception &) {
            cdb_debug("Ignoring potentially unsupported BA_DEF_DEF_");
        }
        phrases.clear();
        numbers.clear();
    };

    std::map<std::uint32_t, std::uint32_t> msgCycleTimes;
    parser["ba_bo"] = [&numbers, &phrases,
                           &msgCycleTimes](const peg::SemanticValues& sv) {
        cdb_debug("Found ba_bo {}", sv.token());
        try {
            auto cycleTime = static_cast<std::uint32_t>(take_back(numbers));
            auto id = static_cast<std::uint32_t>(take_back(numbers));
            auto attributeName = take_back(phrases);
            if (attributeName == "GenMsgCycleTime") {
                msgCycleTimes[id] = cycleTime;
                cdb_debug("Found message cycle time id={}, time={}", id,
                    cycleTime);
            }
        } catch (const std::exception &) {
            cdb_debug("Ignoring potentially unsupported BA_ BO_");
        }
        phrases.clear();
        numbers.clear();
    };

    std::map<std::uint32_t, std::tuple<std::string,
        std::uint32_t>> sigStartValues;
    parser["ba_sg"] = [&numbers, &idents, &phrases,
                           &sigStartValues](const peg::SemanticValues& sv) {
        cdb_debug("Found ba_sg {}", sv.token());
        if (numbers.size() == 2) {
            auto value = static_cast<std::uint32_t>(take_back(numbers));
            auto name = take_back(idents);
            auto id = static_cast<std::uint32_t>(take_back(numbers));
            auto attributeName = take_back(phrases);
            if (attributeName == "GenSigStartValue") {
                sigStartValues[id] = { name, value };
                cdb_debug("Found signal start value id={}, name={}, value={}",
                    id, name, value);
            }
        } else if (phrases.size() == 2) {
            auto value = take_back(phrases);
            auto name = take_back(idents);
            auto id = static_cast<std::uint32_t>(take_back(numbers));
            auto attributeName = take_back(phrases);
            if (attributeName == "GenSigStartValue") {
                // TODO: support strings as signal start values
                cdb_warn("String-based signal start values are not yet " \
                    "supported.");
                cdb_debug("Found signal start value id={}, name={}, value={}",
                    id, name, value);
            }
        } else {
            cdb_debug("Ignoring potentially unsupported BA_ SG_");
        }
        phrases.clear();
        numbers.clear();
        idents.clear();
    };

    return parser.parse(noTabsData.c_str());
}

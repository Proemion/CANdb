#include <gtest/gtest.h>

#include <fstream>

#include "dbcparser.h"
#include "log.hpp"

std::string loadDBCFile(const std::string& filename)
{
    const std::string path = std::string{ EXTENDED_DBC_DIR } + filename;

    std::fstream file{ path.c_str() };

    std::string buff;
    std::copy(std::istreambuf_iterator<char>(file),
        std::istreambuf_iterator<char>(), std::back_inserter(buff));

    file.close();
    return buff;
}

std::shared_ptr<spdlog::logger> kDefaultLogger
    = []() -> std::shared_ptr<spdlog::logger> {
    auto z = std::getenv("CDB_LEVEL");
    auto logger = spdlog::stdout_color_mt("cdb");

    if (z == nullptr) {
        logger->set_level(spdlog::level::err);
    } else {
        const std::string ll{ z };

        auto it = std::find_if(std::begin(spdlog::level::level_names),
            std::end(spdlog::level::level_names),
            [&ll](const char* name) { return std::string{ name } == ll; });

        if (it != std::end(spdlog::level::level_names)) {
            int i = std::distance(std::begin(spdlog::level::level_names), it);
            logger->set_level(static_cast<spdlog::level::level_enum>(i));
        }
    }

    return logger;
}();

struct ExtendedDBCTest : public ::testing::TestWithParam<std::string> {
    CANdb::DBCParser parser;
};

TEST_P(ExtendedDBCTest, parse_dbc_file)
{
    auto dbc_file = GetParam();
    auto file = loadDBCFile(dbc_file);
    ASSERT_TRUE(parser.parse(file));

    if (dbc_file == "extended_example.dbc") {
        auto db = parser.getDb();

        // Check global data for this database
        EXPECT_EQ(db.genMsgCycleTimeMin.get(), 0);
        EXPECT_EQ(db.genMsgCycleTimeMax.get(), 65535);
        EXPECT_EQ(db.genSigStartValueMin.get(), 0);
        EXPECT_EQ(db.genSigStartValueMax.get(), 0);
        EXPECT_EQ(db.genMsgCycleTimeDefault.get(), 0);
        EXPECT_EQ(db.genSigStartValueDefault.get(), 0);

        // Define expected messages and signals for this DBC file
        std::map<CANmessage, std::vector<CANsignal>> expectedMessages
            {
                { CANmessage{2364473086, "VECTOR_INDEPENDENT_SIG_MSG", 2, {},
                    boost::none, std::string("This is a message for not used " \
                    "signals, created by Vector CANdb++ DBC OLE DB Provider.")},
                    { CANsignal{"SENSOR_SONARS_mux", 0, 4,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          0, 0, "", {}},
                      CANsignal{"SENSOR_SONARS_err_count", 0, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          0, 0, "", {}},
                      CANsignal{"SENSOR_SONARS_left", 0, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {}},
                      CANsignal{"SENSOR_SONARS_middle", 0, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {}},
                      CANsignal{"SENSOR_SONARS_right", 0, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {}},
                      CANsignal{"SENSOR_SONARS_rear", 0, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {}},
                      CANsignal{"SENSOR_SONARS_no_filt_left", 0, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {}},
                      CANsignal{"SENSOR_SONARS_no_filt_middle", 0, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {}},
                      CANsignal{"SENSOR_SONARS_no_filt_right", 0, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {}},
                      CANsignal{"SENSOR_SONARS_no_filt_rear", 0, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {}}
                    } },
                { CANmessage{100, "DRIVER_HEARTBEAT", 8, {"DRIVER"}, 1000,
                    std::string("Sync message used to synchronize the " \
                    "controllers")},
                    { CANsignal{"DRIVER_HEARTBEAT_cmd", 0, 8,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          0, 255, "", {"SENSOR", "MOTOR"},
                          CANsignalMuxType::NotMuxed, boost::none,
                          boost::any(3.0), boost::none, boost::none,
                          std::string("2 DRIVER_HEARTBEAT_cmd_REBOOT 1 " \
                            "DRIVER_HEARTBEAT_cmd_SYNC 0 " \
                            "DRIVER_HEARTBEAT_cmd_NOOP")},
                      CANsignal{"TEST_123", 65, 16,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          0, 0, "", {"SENSOR", "MOTOR"}}
                    } },
                { CANmessage{500, "IO_DEBUG", 4, {"IO", "DRIVER"}, 100,
                    boost::none},
                    { CANsignal{"IO_DEBUG_test_unsigned", 0, 8,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          0, 255, "", {"DBG"},
                          CANsignalMuxType::NotMuxed, boost::none,
                          boost::any(12.0)},
                      CANsignal{"IO_DEBUG_test_enum", 8, 8,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          0, 0, "", {"DBG"},
                          CANsignalMuxType::NotMuxed, boost::none,
                          boost::none, boost::none, boost::none,
                          std::string("2 IO_DEBUG_test2_enum_two 1 " \
                            "IO_DEBUG_test2_enum_one")},
                      CANsignal{"IO_DEBUG_test_signed", 17, 7,
                          CANsignalEndianness::LittleEndianIntel, true, 1.5, 0,
                          -96, 94.5, "", {"DBG"},
                          CANsignalMuxType::NotMuxed, boost::none,
                          boost::any(0.0)},
                      CANsignal{"IO_DEBUG_test_float", 24, 8,
                          CANsignalEndianness::LittleEndianIntel, true, 0.5, 0,
                          0, 127.5, "", {"DBG"},
                          CANsignalMuxType::NotMuxed, boost::none,
                          boost::any(0.0)}
                    } },
                { CANmessage{101, "MOTOR_CMD", 2, {"DRIVER"}, 100, boost::none},
                    { CANsignal{"MOTOR_CMD_steer", 0, 4,
                          CANsignalEndianness::LittleEndianIntel, true, 1, -5,
                          -5, 5, "", {"MOTOR"}},
                      CANsignal{"MOTOR_CMD_drive", 4, 4,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          0, 9, "", {"MOTOR"}},
                      CANsignal{"MOTOR_STATUS_wheel_error", 8, 1,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          0, 1, "", {"DRIVER", "IO"}}
                    } },
                { CANmessage{400, "MOTOR_STATUS", 8, {"MOTOR"}, 100,
                    std::string("Comment on message")},
                    { CANsignal{"MOTOR_STATUS_wheel_error", 0, 1,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          0, 1, "", {"DRIVER", "IO"}},
                      CANsignal{"MOTOR_STATUS_speed_kph", 6, 16,
                          CANsignalEndianness::LittleEndianIntel, true, 1, 0,
                          -32768, 32767, "kph", {"DRIVER", "IO"},
                          CANsignalMuxType::NotMuxed, boost::none,
                          boost::none, std::string("Comment on signal")}
                    } },
                { CANmessage{123, "FUEL_STATUS", 8, {"MOTOR"}, boost::none,
                    boost::none},
                    { CANsignal{"FUEL_STATUS_level", 0, 32,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          -3.4E+038, 3.4E+038, "", {"DRIVER", "IO"},
                          CANsignalMuxType::NotMuxed, boost::none,
                          boost::none, boost::none, CANsignalType::Float,
                          boost::none}
                    } },
                { CANmessage{200, "SENSOR_SONARS", 8, {"SENSOR"}, boost::none,
                    boost::none},
                    { CANsignal{"SENSOR_SONARS_mux", 0, 4,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          0, 0, "", {"DRIVER", "IO"}, CANsignalMuxType::Muxer},
                      CANsignal{"SENSOR_SONARS_err_count", 4, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          0, 0, "", {"DRIVER", "IO"}},
                      CANsignal{"SENSOR_SONARS_left", 16, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DRIVER", "IO"}, CANsignalMuxType::Muxed,
                          0},
                      CANsignal{"SENSOR_SONARS_middle", 28, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DRIVER", "IO"}, CANsignalMuxType::Muxed,
                          0},
                      CANsignal{"SENSOR_SONARS_right", 40, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DRIVER", "IO"}, CANsignalMuxType::Muxed,
                          0},
                      CANsignal{"SENSOR_SONARS_rear", 52, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DRIVER", "IO"}, CANsignalMuxType::Muxed,
                          0},
                      CANsignal{"SENSOR_SONARS_no_filt_left", 16, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DBG"}, CANsignalMuxType::Muxed, 1},
                      CANsignal{"SENSOR_SONARS_no_filt_middle", 28, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DBG"}, CANsignalMuxType::Muxed, 1},
                      CANsignal{"SENSOR_SONARS_no_filt_right", 40, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DBG"}, CANsignalMuxType::Muxed, 1},
                      CANsignal{"SENSOR_SONARS_no_filt_rear", 52, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DBG"}, CANsignalMuxType::Muxed, 1}
                    } },
                { CANmessage{222, "SENSOR_TWO_SONARS", 8, {"SENSOR"},
                    boost::none, boost::none},
                    { CANsignal{"SENSOR_TWO_SONARS_mux", 0, 4,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          0, 0, "", {"DRIVER", "IO"}, CANsignalMuxType::Muxer},
                      CANsignal{"SENSOR_TWO_SONARS_err_count", 4, 44,
                          CANsignalEndianness::LittleEndianIntel, false, 1, 0,
                          0, 0, "", {"DRIVER", "IO"}},
                      CANsignal{"SENSOR_TWO_SONARS_left", 48, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DRIVER", "IO"}, CANsignalMuxType::Muxed,
                          0},
                      CANsignal{"SENSOR_TWO_SONARS_middle", 60, 4,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DRIVER", "IO"}, CANsignalMuxType::Muxed,
                          0},
                      CANsignal{"SENSOR_TWO_SONARS_no_filt_left", 48, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DBG"}, CANsignalMuxType::Muxed,
                          1},
                      CANsignal{"SENSOR_TWO_SONARS_no_filt_middle", 60, 4,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DBG"}, CANsignalMuxType::Muxed,
                          1},
                      CANsignal{"SENSOR_TWO_SONARS_hier", 48, 12,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DRIVER", "IO"}, CANsignalMuxType::Muxed,
                          55},
                      CANsignal{"SENSOR_TWO_SONARS_da", 60, 4,
                          CANsignalEndianness::LittleEndianIntel, false, 0.1, 0,
                          0, 0, "", {"DRIVER", "IO"}, CANsignalMuxType::Muxed,
                          55}
                    } },
                { CANmessage{1845, "TemperatureMsg", 8, {}, boost::none,
                    boost::none},
                    { CANsignal{"MultiplexIndexSignal", 39, 4,
                          CANsignalEndianness::BigEndianMotorola, false, 1, 0,
                          0, 0, "", {}, CANsignalMuxType::Muxer},
                      CANsignal{"NormalSignalAlwaysPresent", 16, 4,
                          CANsignalEndianness::LittleEndianIntel, true, 1, 0,
                          0, 0, "", {}},
                      CANsignal{"TemperatureIndoorsMultiplexed", 7, 16,
                          CANsignalEndianness::BigEndianMotorola, true, 0.1,
                          -40, 0, 0, "degC", {}, CANsignalMuxType::Muxed, 0},
                      CANsignal{"TemperatureOutdoorsMultiplexed", 7, 16,
                          CANsignalEndianness::BigEndianMotorola, true, 0.1,
                          -40, 0, 0, "degC", {}, CANsignalMuxType::Muxed, 3},
                      CANsignal{"TemperatureUndergroundMultiplexd", 7, 16,
                          CANsignalEndianness::BigEndianMotorola, true, 0.1,
                          -40, 0, 0, "degC", {}, CANsignalMuxType::Muxed, 11}
                    } }
            };

        // Ensure we found the expected number of messages
        EXPECT_EQ(db.messages.size(), expectedMessages.size());
        for (auto messageIt = db.messages.begin();
                 messageIt != db.messages.end(); ++messageIt) {
            // Ensure this message was expected
            auto expectedMessageIt =
                expectedMessages.find(CANmessage(messageIt->first.id));
            ASSERT_NE(expectedMessageIt, expectedMessages.end());

            // Ensure this message's data and signal count were as expected
            cdb_debug("Checking message {}: {}", messageIt->first.id,
                messageIt->first.name);
            ASSERT_EQ(messageIt->first.name, expectedMessageIt->first.name);
            ASSERT_EQ(messageIt->first.dlc, expectedMessageIt->first.dlc);
            ASSERT_EQ(messageIt->first.ecus, expectedMessageIt->first.ecus);
            ASSERT_EQ(messageIt->first.cycleTime,
                expectedMessageIt->first.cycleTime);
            ASSERT_EQ(messageIt->first.comment,
                expectedMessageIt->first.comment);
            ASSERT_EQ(messageIt->second.size(),
                expectedMessageIt->second.size());

            // Check data for each signal
            for (auto signalIt = messageIt->second.begin();
                signalIt != messageIt->second.end(); ++signalIt) {
                auto expectedSignalIt =
                    std::find(expectedMessageIt->second.begin(),
                        expectedMessageIt->second.end(), CANsignal(*signalIt));
                ASSERT_NE(expectedSignalIt, expectedMessageIt->second.end());

                auto anyCompare = [](const boost::any& first,
                    const boost::any& second) {
                    using boost::any_cast;

                    bool same = first.empty() == second.empty()
                        && first.type() == second.type();

                    if (same) {
                        if (first.type() == typeid(std::string)) {
                            same = any_cast<std::string>(first)
                                == any_cast<std::string>(second);
                        } else if (first.type() == typeid(std::double_t)) {
                            same = std::fabs(any_cast<std::double_t>(first)
                                - any_cast<std::double_t>(second)) < 1E-05;
                        } else if (first.type() == typeid(std::uint32_t)) {
                            same = any_cast<std::uint32_t>(first)
                                == any_cast<std::uint32_t>(second);
                        }
                    }

                    return same;
                };

                cdb_debug("Checking signal: {}", signalIt->signal_name);
                ASSERT_EQ(signalIt->signal_name, expectedSignalIt->signal_name);
                ASSERT_EQ(signalIt->startBit, expectedSignalIt->startBit);
                ASSERT_EQ(signalIt->signalSize, expectedSignalIt->signalSize);
                ASSERT_EQ(signalIt->endianness, expectedSignalIt->endianness);
                ASSERT_EQ(signalIt->valueSigned, expectedSignalIt->valueSigned);
                ASSERT_DOUBLE_EQ(signalIt->factor, expectedSignalIt->factor);
                EXPECT_DOUBLE_EQ(signalIt->offset, expectedSignalIt->offset);
                EXPECT_DOUBLE_EQ(signalIt->min, expectedSignalIt->min);
                EXPECT_DOUBLE_EQ(signalIt->max, expectedSignalIt->max);
                ASSERT_EQ(signalIt->unit, expectedSignalIt->unit);
                ASSERT_EQ(signalIt->receivers, expectedSignalIt->receivers);
                ASSERT_EQ(signalIt->muxType, expectedSignalIt->muxType);
                ASSERT_EQ(signalIt->muxNdx, expectedSignalIt->muxNdx);
                ASSERT_TRUE(anyCompare(signalIt->startValue,
                    expectedSignalIt->startValue));
                ASSERT_EQ(signalIt->comment, expectedSignalIt->comment);
                ASSERT_EQ(signalIt->valueType, expectedSignalIt->valueType);
                ASSERT_EQ(signalIt->valueDescription,
                    expectedSignalIt->valueDescription);
            }
        }
    }
}

INSTANTIATE_TEST_CASE_P(ExtendedDBC, ExtendedDBCTest,
    ::testing::Values("extended_example.dbc"));

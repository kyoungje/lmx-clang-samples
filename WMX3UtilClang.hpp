// Copyright (c) 2025 Kyoungje Oh
// Licensed under the MIT License. See LICENSE file for details.

#ifndef WMX3UTILCLANG_HPP
#define WMX3UTILCLANG_HPP

#include <string>
#include <iostream>
#include <stdio.h>
#include <vector>
#include <cmath>
#include <fstream>
#include <thread> // For std::this_thread::sleep_for
#include <chrono> // For std::chrono
#include <stdexcept>
#include <atomic> // For std::atomic
#include <mutex> // For std::mutex, std::lock_guard
#include <memory> // For std::unique_ptr

#include "nlohmann/json.hpp"
#include "xtl/xbase64.hpp"
#include "xcpp/xdisplay.hpp"

#include "CoreMotionApi.h"
#include "AdvancedMotionApi.h"
#include "LogApi.h"
#include "WMX3Api.h"

namespace nl = nlohmann;

namespace im
{
    struct image
    {   
        inline image(const std::string& filename)
        {
            auto start_time = std::chrono::steady_clock::now();
            while (!std::ifstream(filename)) {
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (std::chrono::steady_clock::now() - start_time > std::chrono::seconds(5)) {
                    std::cerr << "Error: File could not be opened within 5 seconds." << std::endl;
                    return;
                }
            }
            std::ifstream fin(filename, std::ios::binary);
            m_buffer << fin.rdbuf();
        }
        
        std::stringstream m_buffer;
    };
    
    nl::json mime_bundle_repr(const image& i)
    {
        auto bundle = nl::json::object();
        bundle["image/png"] = xtl::base64encode(i.m_buffer.str());
        return bundle;
    }
}

using namespace wmx3Api;
namespace wmxclang
{
    class LogDataHistory
    {
        std::vector<double> feedbackPositions;
        std::vector<double> feedbackVelocities;
        bool overflowFlag = false;
                
        // Define the increase constant as a static member
        static constexpr size_t IncreaseConstant = wmx3Api::constants::maxLogOutputDataSize;

    public:
        LogDataHistory(uint16_t count = IncreaseConstant)
        {
            feedbackPositions.reserve(count);
            feedbackVelocities.reserve(count);
        }

        void addLogData(double position, double velocity) {
            // Add new data to the vectors
            feedbackPositions.push_back(position);
            feedbackVelocities.push_back(velocity);
    
            // Optionally reserve more space if needed
            if (feedbackPositions.size() == feedbackPositions.capacity()) {
                feedbackPositions.reserve(feedbackPositions.size() + IncreaseConstant);
            }
            if (feedbackVelocities.size() == feedbackVelocities.capacity()) {
                feedbackVelocities.reserve(feedbackVelocities.size() + IncreaseConstant);
            }
        }

        uint16_t getLogDataSize() const {
            return feedbackPositions.size();
        }

        std::pair<double, double> getLogData(uint16_t index) const {
            if (index < feedbackPositions.size()) {
                return std::make_pair(feedbackPositions[index], feedbackVelocities[index]);
            } else {
                return std::make_pair(0.0, 0.0);
            }
        }

        void setOverflowFlag(bool flag) {
            overflowFlag = flag;
        }

        void printLogSummary() const {
            std::cout << "LogDataHistory: " << feedbackPositions.size() << " entries, overflow: " << (overflowFlag?"TRUE":"FALSE") << std::endl;            
        }
    };

    class TemporaryFile {
        std::string filename;
    
    public:
        TemporaryFile(const std::string& name) : filename(name) {}
        ~TemporaryFile() { std::remove(filename.c_str()); }
        const std::string& getName() const { return filename; }
    };

    class WmxUtil
    {
        WMX3Api* m_wmx3Lib;
        std::unique_ptr<wmx3Api::Log> m_pLog;
        int8_t m_curChannel = -1;

        std::atomic<bool> stopFlag{false}; // Stop flag for the worker thread
        std::thread logUpdateThread; // Member variable to store the thread        
        std::unique_ptr<LogDataHistory> logDataHistory; // Dynamically managed LogDataHistory instance
        std::mutex logDataMutex;      // Mutex for thread-safe access to logDataHistory

    public:
        WmxUtil(WMX3Api* wmx3Lib) : m_wmx3Lib(wmx3Lib), m_pLog(std::make_unique<wmx3Api::Log>(wmx3Lib))
        {
            std::cout << "A WmxUtil instance is created." << std::endl;
        }
        
        ~WmxUtil()
        {
            
        }

        bool isAvailableChannel(int channel)
        {
            uint32_t ret;
            wmx3Api::MemoryLogStatus memLogStatus;
            
            // Get the memory log status for the specified channel
            ret = m_pLog->GetMemoryLogStatus(channel, &memLogStatus);
            if (ret != wmx3Api::ErrorCode::None) {
                checkErrorCode("GetMemoryLogStatus", ret);
                return false;
            }

            // Check if the log state is idle and the buffer is not opened
            // REVISIT: The bufferOpened flag is alway TRUE even after closing the buffer.
            if (memLogStatus.logState != LogState::Idle && memLogStatus.bufferOpened) {
                std::cout << "memLogStatus.LogState: " << memLogStatus.logState
                          << ", memLogStatus.bufferOpened: " << memLogStatus.bufferOpened << std::endl;
                return false;
            }

            return true;
        }

        uint8_t checkMemoryLogChannel() 
        {
            int8_t channel = -1;

            // Iterate through channels in reverse order
            for (int curChannel = wmx3Api::constants::maxLogChannel - 1; curChannel >= 1; --curChannel) {
                std::cout << "curChannel: " << curChannel << std::endl;

                if (isAvailableChannel(curChannel)) {
                    uint32_t ret = m_pLog->OpenMemoryLogBuffer(curChannel);
                    if (ret == wmx3Api::ErrorCode::None) {
                        channel = curChannel;
                        break;
                    } else {
                        checkErrorCode("OpenMemoryLogBuffer during checkMemoryLogChannel", ret);
                    }
                }
            }

            return channel;
        }
        
        
        uint16_t collectLogData(int channel, int axis = 0) 
        {
            uint16_t logDataCount = 0;

            // Get memory log data
            CoreMotionLogOutput memLogData;
            uint32_t ret = m_pLog->GetMemoryLogData(channel, &memLogData);
            if (ret != wmx3Api::ErrorCode::None) {
                checkErrorCode("collectLogData", ret);
                return logDataCount;
            }

            logDataCount = memLogData.count;
            // Extract feedback position and velocity
            {
                std::lock_guard<std::mutex> lock(logDataMutex);
                
                if (memLogData.overflowFlag > 0) {
                    logDataHistory->setOverflowFlag(true);
                }

                for (size_t index = 0; index < memLogData.count; ++index) {
                    logDataHistory->addLogData(memLogData.axisData[index][axis].feedbackPos, 
                        memLogData.axisData[index][axis].feedbackVelocity);
                }
            }

            return logDataCount;
        }

        void logUpdateTask(int channel) 
        {
            try {
                while (!stopFlag) {     // Check the stop flag
                    // Update log data
                    uint16_t updatedLogDataSize = collectLogData(channel);                    

                    if (updatedLogDataSize == 0) {
                        std::this_thread::sleep_for(std::chrono::milliseconds(1));
                    }
                } 
            } catch (const std::exception& e) {
                std::cerr << "Exception in logUpdateTask: " << e.what() << std::endl;
            }
            
            std::cout << "MemoryLog thread stopped." << std::endl;
        }
    
        int startLog() 
        {
            uint32_t ret;
            uint8_t maxAxes = 2;

            if (logUpdateThread.joinable()) {
                if (m_curChannel != -1) {
                    stopLog(m_curChannel);  // Stop the existing thread if running
                }                
            }

            // Check for an available memory log channel
            int8_t channel = checkMemoryLogChannel();

            if (channel != -1) {
                wmx3Api::AxisSelection axisSel;
                wmx3Api::MemoryLogOptions memLogOptions;
                
                // Set the number of axes
                axisSel.axisCount = maxAxes;

                // Configure each axis
                for (uint8_t idx = 0; idx < maxAxes; ++idx) {
                    axisSel.axis[idx] = idx;
                }
                memLogOptions.triggerEventCount = 0;

                // CoreMotionLogInput in;
                // //Set the CoreMotion module to log the position command data of axis 0
                // in.axisSelection.axisCount = 1;
                // in.axisSelection.axis[0] = 0;
                // in.axisOptions.commandPos = 1;

                // Set the memory log
                // ret = m_pLog->SetMemoryLog(channel, &in);
                ret = m_pLog->SetMemoryLog(channel, &axisSel, &memLogOptions);
                if (ret != wmx3Api::ErrorCode::None) {
                    checkErrorCode("SetMemoryLog during startLog", ret);
                    return -1;
                }

                // Start the memory log
                ret = m_pLog->StartMemoryLog(channel);
                if (ret != wmx3Api::ErrorCode::None) {
                    checkErrorCode("StartMemoryLog during startLog", ret);
                    return -1;
                }

                m_curChannel = channel;

                // Create or reset the LogDataHistory instance dynamically
                {
                    std::lock_guard<std::mutex> lock(logDataMutex);
                    logDataHistory = std::make_unique<LogDataHistory>(); 
                }

                // Reset the stop flag
                stopFlag = false;
                // Start the worker thread for log update
                logUpdateThread = std::thread(&WmxUtil::logUpdateTask, this, channel);
            }

            return channel;
        }
    
        bool stopLog(int channel) 
        {
            wmx3Api::MemoryLogStatus memLogStatus;
            uint32_t ret;

            ret = m_pLog->GetMemoryLogStatus(channel, &memLogStatus);
            if (ret != wmx3Api::ErrorCode::None) {
                checkErrorCode("GetMemoryLogStatus during stopLog", ret);
                return false;
            }

            if (memLogStatus.logState == LogState::Running) {
                ret = m_pLog->StopMemoryLog(channel);
                if (ret != wmx3Api::ErrorCode::None) {
                    checkErrorCode("StopMemoryLog during stopLog", ret);
                    return false;
                }    
            }

            ret = m_pLog->CloseMemoryLogBuffer(channel);
            if (ret != wmx3Api::ErrorCode::None) {
                checkErrorCode("CloseMemoryLogBuffer during stopLog", ret);
                return false;
            }

            m_curChannel = -1;
            return true;
        }

        void checkErrorCode(const std::string& func, uint32_t errCode) 
        {
            std::string lastErrString;

            if (errCode != wmx3Api::ErrorCode::None) {
                if (errCode >= 0x00000 && errCode <= 0x10000) {
                    lastErrString = "function: " + func + ", ErrorType: WMX3Api, ErrorCode: " + std::to_string(errCode);
                } else if (errCode >= 0x11000 && errCode <= 0x11FFF) {
                    lastErrString = "function: " + func + ", ErrorType: Log, ErrorCode: " + std::to_string(errCode);
                } else {
                    lastErrString = "function: " + func + ", ErrorType: Undefined, ErrorCode: " + std::to_string(errCode);
                }

                std::cerr << lastErrString << std::endl;
                throw std::runtime_error(lastErrString);
            }
        }

        void pauseLog(int channel) 
        {
            uint32_t ret;

            if (m_curChannel != channel)
            {
                std::cerr << "Error: The specified channel is not the current channel." << std::endl;
                return;
            }

            // Signal the worker thread to stop
            stopFlag = true;

            // Stop the memory log
            ret = m_pLog->StopMemoryLog(channel);
            if (ret != wmx3Api::ErrorCode::None) {
                checkErrorCode("StopMemoryLog in pauseLog()", ret);
                return;
            }
            
            // To schedule worker thread to stop
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
                            
            // Wait for completion of the worker thread
            if (logUpdateThread.joinable()) {
                logUpdateThread.join(); // Wait for the worker thread to complete
            }

            // Print the summary of updated logdata
            {
                std::lock_guard<std::mutex> lock(logDataMutex);
                if (logDataHistory) {
                    logDataHistory->printLogSummary();
                } else {
                    std::cout << "No log data available." << std::endl;
                }

            }         
        }
    
        void drawPlots(const std::string& title, const std::string& fileName) 
        {
            TemporaryFile posDatafile("rawdata.dat");

            std::cout << "Creating a plot as " << fileName << std::endl;

            // Delete the file if it already exists
            std::remove(fileName.c_str());
            
            std::ofstream posFile(posDatafile.getName());
            if (!posFile.is_open() ) { 
                std::cerr << "Error: Could not open temporary files for plotting." << std::endl;
                return;
            }

            // Write log data to temporary files
            {
                std::lock_guard<std::mutex> lock(logDataMutex);               
                uint16_t logDataSize = logDataHistory->getLogDataSize();
                for (size_t index = 0; index < logDataSize; ++index) {
                    auto cycleData = logDataHistory->getLogData(index);                    
                    posFile << index << " " << cycleData.first << " " << index << " " << cycleData.second << "\n";                    
                }
            }
            posFile.close();
            
            // Open a pipe to gnuplot
            FILE* gp = popen("gnuplot -persist", "w");
            if (!gp) {
                std::cerr << "Error: Could not open a pipe to gnuplot." << std::endl;
                return;
            }

            // Set the terminal to PNG and specify the output file
            fprintf(gp, "set terminal pngcairo size 1600,600 enhanced font 'Verdana,10'\n");
            fprintf(gp, "set output '%s'\n", fileName.c_str());

            // Set multiplot layout for two subplots in a row
            fprintf(gp, "set multiplot layout 1,2 title '%s' font 'Verdana,16'\n", title.c_str());

            // First subplot: Feedback Position
            fprintf(gp, "set xlabel 'Cycle'\n");
            fprintf(gp, "set ylabel 'Position'\n");
            fprintf(gp, "plot 'rawdata.dat' using 1:2 with linespoints linecolor rgb '#32CD32' title 'Feedback Position'\n");

            // Second subplot: Feedback Velocity
            fprintf(gp, "set xlabel 'Cycle'\n");
            fprintf(gp, "set ylabel 'Velocity'\n");
            fprintf(gp, "plot 'rawdata.dat' using 3:4 with linespoints linecolor rgb '#EE82EE' title 'Feedback Velocity'\n");

            // End multiplot
            fprintf(gp, "unset multiplot\n");

            // Flush and close the pipe
            fflush(gp);
            pclose(gp);

            std::cout << "Plot saved as " << fileName << std::endl;
        }
    };
}

#endif // WMX3UTILCLANG_HPP
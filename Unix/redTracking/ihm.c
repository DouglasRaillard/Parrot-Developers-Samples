/*
    Copyright (C) 2014 Parrot SA

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Parrot nor the names
      of its contributors may be used to endorse or promote products
      derived from this software without specific prior written
      permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
    OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
    AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
    OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
    SUCH DAMAGE.
*/
/**
 * @file ihm.c
 * @brief This file contains sources about ncurses IHM used by arsdk example "JumpingSumoPiloting"
 * @date 15/01/2015
 */

/*****************************************
 *
 *             include file :
 *
 *****************************************/
#include "MeasuredData.h"
#include "Logging.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <curses.h>
#include <string.h>
#include <sys/time.h>

#include <libARSAL/ARSAL.h>

#include "ihm.h"


/*****************************************
 *
 *             define :
 *
 *****************************************/

#define HEADER_X 0
#define HEADER_Y 0

#define INFO_X 0
#define INFO_Y 2

#define BATTERY_X 0
#define BATTERY_Y 4

#define ATTITUDE_ROLL_X 0
#define ATTITUDE_ROLL_Y 6
#define ATTITUDE_PITCH_X 0
#define ATTITUDE_PITCH_Y 7
#define ATTITUDE_YAW_X 0
#define ATTITUDE_YAW_Y 8

#define SPEEDX_X 0
#define SPEEDX_Y 10
#define SPEEDY_X 0
#define SPEEDY_Y 11
#define SPEEDZ_X 0
#define SPEEDZ_Y 12

#define GPS_LATITUDE_X 0
#define GPS_LATITUDE_Y 14
#define GPS_LONGITUDE_X 0
#define GPS_LONGITUDE_Y 15
#define GPS_ALTITUDE_X 0
#define GPS_ALTITUDE_Y 16

#define COMMAND_X 0
#define COMMAND_Y 18

#define MEASURED_DATA1_X 0
#define MEASURED_DATA1_Y 19

#define ALTITUDE_X 0
#define ALTITUDE_Y 20

/*****************************************
 *
 *       treshold following flight:
 *
 ****************************************/
#define centerX 300
#define maxX 600
#define maxY 600


#define thresholdRight (centerX+100)
#define thresholdLeft (centerX-100)
#define proportionalThresholdRight (centerX+60)
#define proportionalThresholdLeft (centerX-60)

#define yawProportionalCommandGain 70
#define linearSpeedGain 50
#define verticalSpeedGain 10
#define maxSpeedTargetArea ((maxX/10)*(maxY/10))

#define DATA_X 0
#define DATA_Y 19

/*****************************************
 *
 *             private header:
 *
 ****************************************/
void *IHM_InputProcessing(void *data);

/*****************************************
 *
 *             implementation :
 *
 *****************************************/

IHM_t *IHM_New (IHM_onInputEvent_t onInputEventCallback)
{
    int failed = 0;
    IHM_t *newIHM = NULL;

    // check parameters
    if (onInputEventCallback == NULL)
    {
        failed = 1;
    }

    if (!failed)
    {
        //  Initialize IHM
        newIHM = (IHM_t*)malloc(sizeof(IHM_t));
        if (newIHM != NULL)
        {
            //  Initialize ncurses
            newIHM->mainWindow = initscr();
            newIHM->inputThread = NULL;
            newIHM->run = 1;
            newIHM->onInputEventCallback = onInputEventCallback;
            newIHM->customData = NULL;
        }
        else
        {
            failed = 1;
        }
    }

    if (!failed)
    {
        raw();                  // Line buffering disabled
        keypad(stdscr, TRUE);
        noecho();               // Don't echo() while we do getch
        timeout(100);

        refresh();
    }

    if (!failed)
    {
        //start input thread
        if(ARSAL_Thread_Create(&(newIHM->inputThread), IHM_InputProcessing, newIHM) != 0)
        {
            failed = 1;
        }
    }

    if (failed)
    {
        IHM_Delete (&newIHM);
    }

    return  newIHM;
}

void IHM_Delete (IHM_t **ihm)
{
    //  Clean up

    if (ihm != NULL)
    {
        if ((*ihm) != NULL)
        {
            (*ihm)->run = 0;

            if ((*ihm)->inputThread != NULL)
            {
                ARSAL_Thread_Join((*ihm)->inputThread, NULL);
                ARSAL_Thread_Destroy(&((*ihm)->inputThread));
                (*ihm)->inputThread = NULL;
            }

            delwin((*ihm)->mainWindow);
            (*ihm)->mainWindow = NULL;
            endwin();
            refresh();

            free (*ihm);
            (*ihm) = NULL;
        }
    }
}

void IHM_setCustomData(IHM_t *ihm, void *customData)
{
    if (ihm != NULL)
    {
        ihm->customData = customData;
    }
}

void *IHM_InputProcessing(void *data)
{
    IHM_t *ihm = (IHM_t *) data;
    int key = 0;

    struct timeval beginAutomationTime;
    bool automationActive = false;
    bool followingActive = false;
    COMMAND_STATE state = STATE_NONE;
    int temp = 0;

    if (ihm != NULL)
    {
        while (ihm->run)
        {
            key = getch();
            IHM_PrintMeasuredData(ihm);

            if ((key == 27) || (key =='q'))
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    automationActive = false;
                    followingActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_EXIT, ihm->customData);
                }
            }
            else if(key == KEY_UP)
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    automationActive = false;
                    followingActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_UP, ihm->customData);
                }
            }
            else if(key == KEY_DOWN)
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    automationActive = false;
                    followingActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_DOWN, ihm->customData);
                }
            }
            else if(key == KEY_LEFT)
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    automationActive = false;
                    followingActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_LEFT, ihm->customData);
                }
            }
            else if(key == KEY_RIGHT)
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    automationActive = false;
                    followingActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_RIGHT, ihm->customData);
                }
            }
            else if(key == 'e')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    automationActive = false;
                    followingActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_EMERGENCY, ihm->customData);
                }
            }
            else if(key == 't')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    automationActive = false;
                    followingActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_TAKEOFF, ihm->customData);
                }
            }
            else if(key == ' ')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    automationActive = false;
                    followingActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_LAND, ihm->customData);
                }
            }
            else if(key == 'r')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    automationActive = false;
                    followingActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_FORWARD, ihm->customData);
                }
            }
            else if(key == 'f')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    automationActive = false;
                    followingActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_BACK, ihm->customData);
                }
            }
            else if(key == 'd')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    automationActive = false;
                    followingActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_ROLL_LEFT, ihm->customData);
                }
            }
            else if(key == 'g')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    automationActive = false;
                    followingActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_ROLL_RIGHT, ihm->customData);
                }
            }
            else if(key == 'o')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    gettimeofday(&beginAutomationTime, NULL);
                    automationActive = true;
                    followingActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_TAKEOFF, ihm->customData);
                }
            }
            else if(key == 'i')
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    followingActive = true;
                    automationActive = false;
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_TAKEOFF, ihm->customData);
                    //STATE_DIAGRAM:WAITING_KEY;
                    //STATE_DIAGRAM:WAITING_KEY -> STAB [label="When i key is pressed in IHM"];
                    state = STATE_STAB;
                    temp = 0;
                }
            }
            else
            {
                if(ihm->onInputEventCallback != NULL)
                {
                    if (automationActive == true)
                    {
                        followingActive = false;
                        AutonomousNavigation(ihm, beginAutomationTime, &automationActive);
                    }
                    else if (followingActive == true)
                    {
                        automationActive = false;
                        //ihm->onInputEventCallback (IHM_INPUT_EVENT_FORWARD, ihm->customData);
                        FollowingNavigation(ihm, &followingActive, &state, &temp);
                    }
                    else
                    {
                        ihm->onInputEventCallback (IHM_INPUT_EVENT_NONE, ihm->customData);
                    }
                }
            }

            dumpLog();
            usleep(10);
        }
    }

    return NULL;
}

void AutonomousNavigation(IHM_t *ihm, struct timeval beginTime, bool *automationActive)
{
    if(automationActive)
    {
        struct timeval currentTime;

        gettimeofday(&currentTime, NULL);
        if(((currentTime.tv_sec*1000 + currentTime.tv_usec/1000) - (beginTime.tv_sec*1000 + beginTime.tv_usec/1000)) < 2000)
        {
            ihm->onInputEventCallback (IHM_INPUT_EVENT_UP, ihm->customData);
        }
        if((((currentTime.tv_sec*1000 + currentTime.tv_usec/1000) - (beginTime.tv_sec*1000 + beginTime.tv_usec/1000)) >= 2000)
            && (((currentTime.tv_sec*1000 + currentTime.tv_usec/1000) - (beginTime.tv_sec*1000 + beginTime.tv_usec/1000)) < 4000))
        {
            ihm->onInputEventCallback (IHM_INPUT_EVENT_FORWARD, ihm->customData);
        }
        if((((currentTime.tv_sec*1000 + currentTime.tv_usec/1000) - (beginTime.tv_sec*1000 + beginTime.tv_usec/1000)) >= 4000)
            && (((currentTime.tv_sec*1000 + currentTime.tv_usec/1000) - (beginTime.tv_sec*1000 + beginTime.tv_usec/1000)) < 5000))
        {
            ihm->onInputEventCallback (IHM_INPUT_EVENT_NONE, ihm->customData);
        }
        if(((currentTime.tv_sec*1000 + currentTime.tv_usec/1000) - (beginTime.tv_sec*1000 + beginTime.tv_usec/1000)) >= 5000)
        {
            ihm->onInputEventCallback (IHM_INPUT_EVENT_LAND, ihm->customData);
            *automationActive = false;
        }
    }

}

void FollowingNavigation(IHM_t *ihm, bool *followingActive, COMMAND_STATE *state, int *temp)
{
    if(true)
    {
        MEASURED_DATA_T trackPoints = redtracking_get_measured_data();;
        ARCONTROLLER_Device_t *deviceController = (ARCONTROLLER_Device_t *)ihm->customData;

        // If not track points, find one
        if(trackPoints.centers.empty()) {
            ihm->onInputEventCallback (IHM_INPUT_EVENT_NONE, ihm->customData);
            //STATE_DIAGRAM:NONE -> INITIAL_SEARCH [label="When no tracked points are in sight"];
            //STATE_DIAGRAM:FOLLOW -> INITIAL_SEARCH [label="When no tracked points are in sight"];
            //STATE_DIAGRAM:SEARCH -> INITIAL_SEARCH [label="When no tracked points are in sight"];

            *state = STATE_INITIAL_SEARCH; // If no target is found, keep searching for it
        }

        //Print state
        move(DATA_Y, 0);     // move to begining of line
        clrtoeol();          // clear line
        mvprintw(DATA_Y, DATA_X, "State: %u", *state);

        switch(*state)
        {
            case STATE_NONE:
                ihm->onInputEventCallback (IHM_INPUT_EVENT_NONE, ihm->customData);
                break;

            case STATE_STAB:
                ihm->onInputEventCallback (IHM_INPUT_EVENT_NONE, ihm->customData);
                (*temp)++;
                if(*temp > 20)
                {
                    *state = STATE_INITIAL_SEARCH; //Start the initial target scan after 20 cycle
                    *temp = 0;
                }
                break;

            case STATE_INITIAL_SEARCH:
                if(!trackPoints.centers.empty())
                {
                    // Threshold controller
                    if(trackPoints.centers[0].first < thresholdLeft){
                        ihm->onInputEventCallback (IHM_INPUT_EVENT_LEFT, ihm->customData);
                    }
                    else if(trackPoints.centers[0].first > thresholdRight) {
                        ihm->onInputEventCallback (IHM_INPUT_EVENT_RIGHT, ihm->customData);
                    }
                    else
                    {
                        *state = STATE_SEARCH; // Start search (tracking controller) if the target is approximately in front of the drone
                        *temp = 0;
                    }
                }
                else {
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_RIGHT, ihm->customData);
                }
                break;

            case STATE_FOLLOW:
                {
                    //ihm->onInputEventCallback (IHM_INPUT_EVENT_FORWARD, ihm->customData);
                    unsigned long speed = linearSpeedGain*(maxSpeedTargetArea/trackPoints.areas[0]);
                    deviceController->aRDrone3->setPilotingPCMDPitch(deviceController->aRDrone3, speed);
                    deviceController->aRDrone3->setPilotingPCMDFlag(deviceController->aRDrone3, 1);

                    (*temp)++;
                    // Search the target once in a while, or do it if absolutely necessary (target really not in the front)
                    //if (*temp > 20000 || trackPoints.centers[0].first < thresholdLeft || trackPoints.centers[0].first > thresholdRight)
                    if (trackPoints.centers[0].first < thresholdLeft || trackPoints.centers[0].first > thresholdRight)
                    {
                        *state = STATE_SEARCH; // Reenable if the target is really not in front of the drone
                        *temp = 0;
                    }
                    break;
                }

            case STATE_SEARCH:
                {
                    // New Proportionnal controller
                    int roll_speed_command = 0;
                    int pitch_speed_command = 0;
                    int yaw_speed_command = yawProportionalCommandGain*(trackPoints.centers[0].first-centerX)/(double)(maxX-centerX);

                    // If approximately correctly oriented, goes forward, else only orientate
                    if(trackPoints.centers[0].first > proportionalThresholdLeft && trackPoints.centers[0].first < proportionalThresholdRight){
                        //pitch_speed_command = linearSpeedGain;
                        double linear_speed_factor = maxSpeedTargetArea/trackPoints.areas[0];
                        if(linear_speed_factor >= 1.0){
                            linear_speed_factor = 1.0;
                        }
                        pitch_speed_command = linearSpeedGain*linear_speed_factor;
                    }

                    // Sign opposed to sign of yaw command
                    int vertical_speed_sign = yaw_speed_command >= 0 ? -1 : 1;
                    int vertical_speed_command = vertical_speed_sign*verticalSpeedGain;
                    bool roll_or_pitch = pitch_speed_command != 0 || roll_speed_command != 0;
                    deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, roll_or_pitch, roll_speed_command, pitch_speed_command, yaw_speed_command, vertical_speed_command, 0);
                }
                /*
                // Old PI controller
                // If approximately correctly oriented, goes straight
                if(trackPoints.centers[0].first > proportionalThresholdLeft && trackPoints.centers[0].first < proportionalThresholdRight){
                    *state = STATE_FOLLOW; // If the target is right ahead, go straight to it without the proportionnal controller
                    *temp = 0;
                }
                // If too much deviation, use the proportionnal corrector at a slower linear speed
                else
                {
                    int yaw_command = yawProportionalCommandGain*(trackPoints.centers[0].first-centerX)/(maxX-centerX);
                    // Sign opposed to sign of yaw command
                    int vertical_speed_sign = yaw_command >= 0 ? -1 : 1;
                    deviceController->aRDrone3->setPilotingPCMD(deviceController->aRDrone3, 0, 0, 0, yaw_command, vertical_speed_sign*10, 0);
                }
                */

                /*
                // Threshold controller
                if(trackPoints.centers[0].first < thresholdLeft)
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_LEFT, ihm->customData);
                else if(trackPoints.centers[0].first > thresholdRight)
                    ihm->onInputEventCallback (IHM_INPUT_EVENT_RIGHT, ihm->customData);
                else
                {
                    // *state = STATE_FOLLOW;
                    *temp = 0;
                }
                */

                break;

            case STATE_LANDING:
                ihm->onInputEventCallback (IHM_INPUT_EVENT_LAND, ihm->customData);
                *state = STATE_NONE; // Drone has landed, waiting for orders
                *followingActive = false;
                break;

            default:
                *state = STATE_NONE;
                ihm->onInputEventCallback (IHM_INPUT_EVENT_NONE, ihm->customData);
                *followingActive = false;
                break;
        }
    }
}

void IHM_PrintHeader(IHM_t *ihm, char *headerStr)
{
    if (ihm != NULL)
    {
        move(HEADER_Y, 0);   // move to begining of line
        clrtoeol();          // clear line
        mvprintw(HEADER_Y, HEADER_X, headerStr);
    }
}

void IHM_PrintInfo(IHM_t *ihm, char *infoStr)
{
    if (ihm != NULL)
    {
        move(INFO_Y, 0);    // move to begining of line
        clrtoeol();         // clear line
        mvprintw(INFO_Y, INFO_X, infoStr);
    }
}

void IHM_PrintBattery(IHM_t *ihm, uint8_t percent)
{
    if (ihm != NULL)
    {
        move(BATTERY_Y, 0);     // move to begining of line
        clrtoeol();             // clear line
        mvprintw(BATTERY_Y, BATTERY_X, "Battery: %d", percent);
    }
}

void IHM_PrintAttitude(IHM_t *ihm, float roll, float pitch, float yaw)
{
    if (ihm != NULL)
    {
        move(ATTITUDE_ROLL_Y, 0);     // move to begining of line
        clrtoeol();              // clear line
        mvprintw(ATTITUDE_ROLL_Y, ATTITUDE_ROLL_X, "Roll: %.5f", roll);

        move(ATTITUDE_PITCH_Y, 0);     // move to begining of line
        clrtoeol();              // clear line
        mvprintw(ATTITUDE_PITCH_Y, ATTITUDE_PITCH_X, "Pitch: %.5f", pitch);

        move(ATTITUDE_YAW_Y, 0);     // move to begining of line
        clrtoeol();              // clear line
        mvprintw(ATTITUDE_YAW_Y, ATTITUDE_YAW_X, "Yaw: %.5f", yaw);
    }
}

void IHM_PrintSpeed(IHM_t *ihm, float speedX, float speedY, float speedZ)
{
    if (ihm != NULL)
    {
        move(SPEEDX_Y, 0);     // move to begining of line
        clrtoeol();              // clear line
        mvprintw(SPEEDX_Y, SPEEDX_X, "Speed X: %f", speedX);

        move(SPEEDY_Y, 0);     // move to begining of line
        clrtoeol();              // clear line
        mvprintw(SPEEDY_Y, SPEEDY_X, "Speed Y: %f", speedY);

        move(SPEEDZ_Y, 0);     // move to begining of line
        clrtoeol();              // clear line
        mvprintw(SPEEDZ_Y, SPEEDZ_X, "Speed Z: %f", speedZ);
    }
    addValueForNextLogEntry(FIELD_SPEED_X, speedX);
    addValueForNextLogEntry(FIELD_SPEED_Y, speedY);
    addValueForNextLogEntry(FIELD_SPEED_Z, speedZ);
}

void IHM_PrintPosition(IHM_t *ihm, double latitude, double longitude, double altitude)
{
    if (ihm != NULL)
    {
        move(GPS_LATITUDE_Y, 0);     // move to begining of line
        clrtoeol();              // clear line
        mvprintw(GPS_LATITUDE_Y, GPS_LATITUDE_X, "GPS Latitude: %d", latitude);

        move(GPS_LONGITUDE_Y, 0);     // move to begining of line
        clrtoeol();              // clear line
        mvprintw(GPS_LONGITUDE_Y, GPS_LONGITUDE_X, "GPS Longitude: %d", longitude);

        move(GPS_ALTITUDE_Y, 0);     // move to begining of line
        clrtoeol();              // clear line
        mvprintw(GPS_ALTITUDE_Y, GPS_ALTITUDE_X, "GPS Altitude: %d", altitude);
    }
}

void IHM_PrintAltitude(IHM_t *ihm, double altitude)
{
    if (ihm != NULL)
    {
        move(ALTITUDE_Y, 0);     // move to begining of line
        clrtoeol();              // clear line
        mvprintw(ALTITUDE_Y, ALTITUDE_X, "Altitude: %d", altitude);
    }
}

void IHM_PrintCommand(IHM_t *ihm, int event)
{
    if (ihm != NULL)
    {
        move(COMMAND_Y, 0);     // move to begining of line
        clrtoeol();             // clear line
        switch (event)
        {
            case 0:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: NONE");
                break;
            case 1:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: EXIT");
                break;
            case 2:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: EMERGENCY");
                break;
            case 3:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: TAKEOFF");
                break;
            case 4:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: LAND");
                break;
            case 5:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: UP");
                break;
            case 6:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: DOWN");
                break;
            case 7:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: RIGHT");
                break;
            case 8:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: LEFT");
                break;
            case 9:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: FORWARD");
                break;
            case 10:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: BACK");
                break;
            case 11:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: ROLL_LEFT");
                break;
            case 12:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: ROLL_RIGHT");
                break;
            default:
                mvprintw(COMMAND_Y, COMMAND_X, "Command: ....");
                break;
        }
        addValueForNextLogEntry(FIELD_COMMAND_ID, event);
    }
}

void IHM_PrintMeasuredData(IHM_t *ihm)
{
    if (ihm != NULL)
    {
        MEASURED_DATA_T data = redtracking_get_measured_data();
        move(MEASURED_DATA1_Y, 0);     // move to begining of line
        clrtoeol();              // clear line
        if (!data.centers.empty()) {
            mvprintw(MEASURED_DATA1_Y, MEASURED_DATA1_X, "Trackpoint 1: %u, %u", data.centers[0].first, data.centers[0].second);
        } else {
            mvprintw(MEASURED_DATA1_Y, MEASURED_DATA1_X, "Trackpoint 1: lost");
        }

    }
}
#ifdef __cplusplus
}
#endif

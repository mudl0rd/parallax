/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *   Mupen64plus-core - api/frontend.c                                     *
 *   Mupen64Plus homepage: https://mupen64plus.org/                        *
 *   Copyright (C) 2012 CasualJames                                        *
 *   Copyright (C) 2009 Richard Goedeken                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.          *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
                       
/* This file contains the Core front-end functions which will be exported
 * outside of the core library.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define M64P_CORE_PROTOTYPES 1
#include "callbacks.h"
#include "api/config.h"
#include "m64p_config.h"
#include "m64p_frontend.h"
#include "m64p_types.h"
#include "main/eventloop.h"
#include "main/main.h"
#include "main/rom.h"
#include "main/util.h"
#include "main/version.h"
#include "plugin/plugin.h"
#include "vidext.h"

/* some local state variables */
static int l_CoreInit = 0;
static int l_ROMOpen = 0;
static int l_CallerUsingSDL = 0;

/* functions exported outside of libmupen64plus to front-end application */
EXPORT m64p_error CALL CoreStartup(int APIVersion, const char *ConfigPath, const char *DataPath, void *Context,
                                   void (*DebugCallback)(void *, int, const char *), void *Context2,
                                   void (*StateCallback)(void *, m64p_core_param, int))
{
    if (l_CoreInit)
        return M64ERR_ALREADY_INIT;

    /* check wether the caller has already initialized SDL */
    l_CallerUsingSDL = 1; //(SDL_WasInit(0) != 0);

    /* very first thing is to set the callback functions for debug info and state changing*/
    SetDebugCallback(DebugCallback, Context);
    SetStateCallback(StateCallback, Context2);

    /* check front-end's API version */
    if ((APIVersion & 0xffff0000) != (FRONTEND_API_VERSION & 0xffff0000))
    {
        DebugMessage(M64MSG_ERROR, "CoreStartup(): Front-end (API version %i.%i.%i) is incompatible with this core (API %i.%i.%i)",
                     VERSION_PRINTF_SPLIT(APIVersion), VERSION_PRINTF_SPLIT(FRONTEND_API_VERSION));
        return M64ERR_INCOMPATIBLE;
    }
   
    /* allocate base memory */
    g_mem_base = init_mem_base();
    if (g_mem_base == NULL) {
        return M64ERR_NO_MEMORY;
    }

    /* The ROM database contains MD5 hashes, goodnames, and some game-specific parameters */
    romdatabase_open();

    l_CoreInit = 1;
    return M64ERR_SUCCESS;
}

EXPORT m64p_error CALL CoreShutdown(void)
{
    if (!l_CoreInit)
        return M64ERR_NOT_INIT;

    /* close down some core sub-systems */
    romdatabase_close();
    ConfigShutdown();

    /* deallocate base memory */
    release_mem_base(g_mem_base);
    g_mem_base = NULL;

    l_CoreInit = 0;
    return M64ERR_SUCCESS;
}

EXPORT m64p_error CALL CoreAttachPlugin(m64p_plugin_type PluginType, m64p_dynlib_handle PluginLibHandle)
{
    return M64ERR_SUCCESS;
}

EXPORT m64p_error CALL CoreDetachPlugin(m64p_plugin_type PluginType)
{
    return M64ERR_SUCCESS;
}

EXPORT m64p_error CALL CoreDoCommand(m64p_command Command, int ParamInt, void *ParamPtr)
{
    m64p_error rval;
    int keysym, keymod;

    if (!l_CoreInit)
        return M64ERR_NOT_INIT;

    switch(Command)
    {
        case M64CMD_NOP:
            return M64ERR_SUCCESS;
        case M64CMD_ROM_OPEN:
            if (g_EmulatorRunning || l_ROMOpen)
                return M64ERR_INVALID_STATE;
            if (ParamPtr == NULL || ParamInt < 4096)
                return M64ERR_INPUT_ASSERT;
            rval = open_rom((const unsigned char *) ParamPtr, ParamInt);
            if (rval == M64ERR_SUCCESS)
            {
                l_ROMOpen = 1;
            }
            return rval;
        case M64CMD_ROM_CLOSE:
            if (g_EmulatorRunning || !l_ROMOpen)
                return M64ERR_INVALID_STATE;
            l_ROMOpen = 0;
            return close_rom();
        case M64CMD_PIF_OPEN:
            if (g_EmulatorRunning)
                return M64ERR_INVALID_STATE;
            if (ParamPtr == NULL || ParamInt != 2048)
                return M64ERR_INPUT_ASSERT;
            return open_pif((const unsigned char *) ParamPtr, ParamInt);
        case M64CMD_ROM_GET_HEADER:
            if (!l_ROMOpen)
                return M64ERR_INVALID_STATE;
            if (ParamPtr == NULL)
                return M64ERR_INPUT_ASSERT;
            if (sizeof(m64p_rom_header) < ParamInt)
                ParamInt = sizeof(m64p_rom_header);
            memcpy(ParamPtr, &ROM_HEADER, ParamInt);
            // Mupen64Plus used to keep a m64p_rom_header with a clean ROM name
            // Keep returning a clean ROM name for backwards compatibility
            if (ParamInt >= 0x20)
            {
                int size = (ParamInt >= 0x20 + 20) ? 20 : (ParamInt - 0x20);
                memcpy((char *)ParamPtr + 0x20, ROM_PARAMS.headername, size);
            }
            return M64ERR_SUCCESS;
        case M64CMD_ROM_GET_SETTINGS:
            if (!l_ROMOpen)
                return M64ERR_INVALID_STATE;
            if (ParamPtr == NULL)
                return M64ERR_INPUT_ASSERT;
            if (sizeof(m64p_rom_settings) < ParamInt)
                ParamInt = sizeof(m64p_rom_settings);
            memcpy(ParamPtr, &ROM_SETTINGS, ParamInt);
            return M64ERR_SUCCESS;
        case M64CMD_ROM_SET_SETTINGS:
            if (g_EmulatorRunning || !l_ROMOpen)
                return M64ERR_INVALID_STATE;
            if (ParamPtr == NULL)
                return M64ERR_INPUT_ASSERT;
            if ((int)sizeof(m64p_rom_settings) < ParamInt)
                ParamInt = sizeof(m64p_rom_settings);
            memcpy(&ROM_SETTINGS, ParamPtr, ParamInt);
            return M64ERR_SUCCESS;
        case M64CMD_EXECUTE:
            if (g_EmulatorRunning || !l_ROMOpen)
                return M64ERR_INVALID_STATE;
            /* the main_run() function will not return until the player has quit the game */
            rval = main_run();
            return rval;
        case M64CMD_STOP:
            if (!g_EmulatorRunning)
                return M64ERR_INVALID_STATE;
            /* this stop function is asynchronous.  The emulator may not terminate until later */
            return main_core_state_set(M64CORE_EMU_STATE, M64EMU_STOPPED);
        case M64CMD_PAUSE:
            if (!g_EmulatorRunning)
                return M64ERR_INVALID_STATE;
            return main_core_state_set(M64CORE_EMU_STATE, M64EMU_PAUSED);
        case M64CMD_RESUME:
            if (!g_EmulatorRunning)
                return M64ERR_INVALID_STATE;
            return main_core_state_set(M64CORE_EMU_STATE, M64EMU_RUNNING);
        case M64CMD_SEND_SDL_KEYDOWN:
            if (!g_EmulatorRunning)
                return M64ERR_INVALID_STATE;
            keysym = ParamInt & 0xffff;
            keymod = (ParamInt >> 16) & 0xffff;
            return M64ERR_SUCCESS;
        case M64CMD_SEND_SDL_KEYUP:
            if (!g_EmulatorRunning)
                return M64ERR_INVALID_STATE;
            keysym = ParamInt & 0xffff;
            keymod = (ParamInt >> 16) & 0xffff;
            return M64ERR_SUCCESS;
        case M64CMD_SET_FRAME_CALLBACK:
            g_FrameCallback = (m64p_frame_callback) ParamPtr;
            return M64ERR_SUCCESS;
        case M64CMD_TAKE_NEXT_SCREENSHOT:
            if (!g_EmulatorRunning)
                return M64ERR_INVALID_STATE;
            return M64ERR_SUCCESS;
        case M64CMD_RESET:
            if (!g_EmulatorRunning)
                return M64ERR_INVALID_STATE;
            if (ParamInt < 0 || ParamInt > 1)
                return M64ERR_INPUT_INVALID;
            return main_reset(ParamInt);
        case M64CMD_ADVANCE_FRAME:
            if (!g_EmulatorRunning)
                return M64ERR_INVALID_STATE;
            return M64ERR_SUCCESS;
        default:
            return M64ERR_INPUT_INVALID;
    }

    return M64ERR_INTERNAL;
}

EXPORT m64p_error CALL CoreOverrideVidExt(m64p_video_extension_functions *VideoFunctionStruct)
{
    if (!l_CoreInit)
        return M64ERR_NOT_INIT;

    return OverrideVideoFunctions(VideoFunctionStruct); /* in vidext.c */
}

EXPORT m64p_error CALL CoreAddCheat(const char *CheatName, m64p_cheat_code *CodeList, int NumCodes)
{
return M64ERR_SUCCESS;

}

EXPORT m64p_error CALL CoreCheatEnabled(const char *CheatName, int Enabled)
{
        return M64ERR_SUCCESS;

}

EXPORT m64p_error CALL CoreGetRomSettings(m64p_rom_settings *RomSettings, int RomSettingsLength, int Crc1, int Crc2)
{
    romdatabase_entry* entry;
    int i;

    if (!l_CoreInit)
        return M64ERR_NOT_INIT;
    if (RomSettings == NULL)
        return M64ERR_INPUT_ASSERT;
    if (RomSettingsLength < sizeof(m64p_rom_settings))
        return M64ERR_INPUT_INVALID;

    /* Look up this ROM in the .ini file and fill in goodname, etc */
    entry = ini_search_by_crc(Crc1, Crc2);
    if (entry == NULL)
        return M64ERR_INPUT_NOT_FOUND;

    strncpy(RomSettings->goodname, entry->goodname, 255);
    RomSettings->goodname[255] = '\0';
    for (i = 0; i < 16; i++)
        sprintf(RomSettings->MD5 + i*2, "%02X", entry->md5[i]);
    RomSettings->MD5[32] = '\0';
    RomSettings->savetype = entry->savetype;
    RomSettings->status = entry->status;
    RomSettings->players = entry->players;
    RomSettings->rumble = entry->rumble;
    RomSettings->transferpak = entry->transferpak;
    RomSettings->mempak = entry->mempak;
    RomSettings->disableextramem = entry->disableextramem;
    RomSettings->countperop = entry->countperop;
    RomSettings->savetype = entry->savetype;
    RomSettings->sidmaduration = entry->sidmaduration;

    return M64ERR_SUCCESS;
}



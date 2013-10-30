/*
**                Copyright 2012 by Kvaser AB, M�lndal, Sweden
**                        http://www.kvaser.com
**
** This software is dual licensed under the following two licenses:
** BSD-new and GPLv2. You may use either one. See the included
** COPYING file for details.
**
** License: BSD-new
** ===============================================================================
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
** ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
** DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
** DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
** (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
** LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
** ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
** SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**
**
** License: GPLv2
** ===============================================================================
** This program is free software; you can redistribute it and/or
** modify it under the terms of the GNU General Public License
** as published by the Free Software Foundation; either version 2
** of the License, or (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
**
** ---------------------------------------------------------------------------
**/

/*  Kvaser Linux Canlib LAPcan helper functions */

#ifndef _CANLIB_UTIL_H_
#define _CANLIB_UTIL_H_


#include <lapcmds.h>
#include <lapcan_ioctl.h>
#include <lapcan_data.h>
#include <canlib_data.h>

#include <osif_common.h>

#   include <sys/ioctl.h>

int addFilter (OS_IF_FILE_HANDLE fd,
           unsigned int cmdNr,  int cmdNrMask,
           unsigned char chanNr, char chanNrMask,
           unsigned char flags,  char flagsMask,
           int isPass);
int clearFilters (OS_IF_FILE_HANDLE fd);

OS_IF_INLINE int getCmdNr (lpcCmd *cmd);
OS_IF_INLINE int getCmdLen (lpcCmd *cmd);
OS_IF_INLINE int getCmdChan (lpcCmd *cmd);
OS_IF_INLINE unsigned char getFlags (lpcCmd *cmd);
OS_IF_INLINE void setFlags (lpcCmd *cmd, unsigned char flags);
OS_IF_INLINE void copyLapcanCmd (lpcCmd *cmd_to, lpcCmd *cmd_from);

#endif /* CANLIB_UTIL_H */
############################################################################
#
#   Copyright (c) 2017 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

add_subdirectory(battery EXCLUDE_FROM_ALL)
add_subdirectory(controllib EXCLUDE_FROM_ALL)
add_subdirectory(conversion EXCLUDE_FROM_ALL)
add_subdirectory(DriverFramework/framework EXCLUDE_FROM_ALL)
add_subdirectory(ecl EXCLUDE_FROM_ALL)
add_subdirectory(external_lgpl EXCLUDE_FROM_ALL)
add_subdirectory(hx_stream EXCLUDE_FROM_ALL)
add_subdirectory(led EXCLUDE_FROM_ALL)
add_subdirectory(log EXCLUDE_FROM_ALL)
add_subdirectory(mathlib EXCLUDE_FROM_ALL)
add_subdirectory(mixer EXCLUDE_FROM_ALL)
add_subdirectory(rc EXCLUDE_FROM_ALL)
add_subdirectory(parameters EXCLUDE_FROM_ALL)
add_subdirectory(perf_counter EXCLUDE_FROM_ALL)
add_subdirectory(pid EXCLUDE_FROM_ALL)
add_subdirectory(pwm_limit EXCLUDE_FROM_ALL)
add_subdirectory(terrain_estimation EXCLUDE_FROM_ALL)
add_subdirectory(tinybson EXCLUDE_FROM_ALL)
add_subdirectory(version EXCLUDE_FROM_ALL)

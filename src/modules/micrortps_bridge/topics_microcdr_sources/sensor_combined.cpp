/****************************************************************************
 *
 *   Copyright (C) 2013-2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/* Auto-generated by genmsg_cpp from file /home/dagar/git/PX4-Autopilot_upstream/msg/sensor_combined.msg */


#include <px4_platform_common/px4_config.h>
#include <ucdr/microcdr.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB_microcdr/topics/sensor_combined.h>


void serialize_sensor_combined(ucdrBuffer *writer, const struct sensor_combined_s *input, char *output, uint32_t *length)
{
    if (nullptr == writer || nullptr == input || nullptr == output || nullptr == length)
	return;

    ucdr_reset_buffer(writer);

    ucdr_serialize_uint64_t(writer, input->timestamp);
    ucdr_serialize_array_float(writer, input->gyro_rad, 3);
    ucdr_serialize_uint32_t(writer, input->gyro_integral_dt);
    ucdr_serialize_int32_t(writer, input->accelerometer_timestamp_relative);
    ucdr_serialize_array_float(writer, input->accelerometer_m_s2, 3);
    ucdr_serialize_uint32_t(writer, input->accelerometer_integral_dt);
    ucdr_serialize_uint8_t(writer, input->accelerometer_clipping);

    (*length) = ucdr_buffer_length(writer);
}

void deserialize_sensor_combined(ucdrBuffer *reader, struct sensor_combined_s *output, const char *input)
{
    if (nullptr == reader || nullptr == output || nullptr == input)
	return;

    ucdr_reset_buffer(reader);

    ucdr_deserialize_uint64_t(reader, &output->timestamp);
    ucdr_deserialize_array_float(reader, output->gyro_rad, 3);
    ucdr_deserialize_uint32_t(reader, &output->gyro_integral_dt);
    ucdr_deserialize_int32_t(reader, &output->accelerometer_timestamp_relative);
    ucdr_deserialize_array_float(reader, output->accelerometer_m_s2, 3);
    ucdr_deserialize_uint32_t(reader, &output->accelerometer_integral_dt);
    ucdr_deserialize_uint8_t(reader, &output->accelerometer_clipping);

}

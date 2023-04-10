#pragma once

typedef struct __synchropter_sim_data {
	uint64_t time_usec;
	float attitude_quaternion[4];
	float roll_rate;
	float pitch_rate;
	float yaw_rate;
	float accel_x;
	float accel_y;
	float accel_z;
	float vel_north;
	float vel_east;
	float vel_down;
	int32_t lat;
	int32_t lon;
	float alt;
	float alt_baro;
	float alt_rate;
	float temperature;
	short cas;
	short tas;
	float baro_pressure;
	float alt_agl;

	// float abs_pressure;
	// uint16_t eph;
	// uint16_t epv;
	// int8_t battery_remain;
} synchropter_sim_data_s;

static inline uint64_t UInt64Parse(const uint8_t* data, int index)
{
	uint64_t value = 0;
	memcpy(&value, &data[index], sizeof(uint64_t));
	return value;
}

static inline float FloatParse(const uint8_t* data, int index)
{
	float value = 0.0f;
	memcpy(&value, &data[index], sizeof(float));
	return value;
}

static inline short ShortParse(const uint8_t* data, int index)
{
	short value = 0;
	memcpy(&value, &data[index], sizeof(short));
	return value;
}

static inline int32_t Int32Parse(const uint8_t* data, int index)
{
	int32_t value = 0;
	memcpy(&value, &data[index], sizeof(int32_t));
	return value;
}

static inline int16_t Int16Parse(const uint8_t* data, int index)
{
	int16_t value = 0;
	memcpy(&value, &data[index], sizeof(int16_t));
	return value;
}

static inline int8_t Int8Parse(const uint8_t* data, int index)
{
	int8_t value = 0;
	memcpy(&value, &data[index], sizeof(int8_t));
	return value;
}

static inline uint16_t UInt16Parse(const uint8_t* data, int index)
{
	uint16_t value = 0;
	memcpy(&value, &data[index], sizeof(uint16_t));
	return value;
}

static inline void FloatArrayParse(const uint8_t* data, float* array, uint16_t array_length, int index)
{
	memcpy(array, &data[index], sizeof(float) * array_length);
	return;
}

static inline uint64_t mavlink_synchropter_sim_data_get_time_usec(const uint8_t* data)
{
	return UInt64Parse(data, 0);
}

static inline void mavlink_synchropter_sim_data_get_attitude_q(const uint8_t* data, float* attitude_q)
{
	FloatArrayParse(data, attitude_q, sizeof(float) * 4, 8);
}

static inline float mavlink_synchropter_sim_data_get_roll_rate(const uint8_t* data)
{
	return FloatParse(data, 24);
}

static inline float mavlink_synchropter_sim_data_get_pitch_rate(const uint8_t* data)
{
	return FloatParse(data, 28);
}

static inline float mavlink_synchropter_sim_data_get_yaw_rate(const uint8_t* data)
{
	return FloatParse(data, 32);
}

static inline float mavlink_synchropter_sim_data_get_accel_x(const uint8_t* data)
{
	return FloatParse(data, 36);
}

static inline float mavlink_synchropter_sim_data_get_accel_y(const uint8_t* data)
{
	return FloatParse(data, 40);
}

static inline float mavlink_synchropter_sim_data_get_accel_z(const uint8_t* data)
{
	return FloatParse(data, 44);
}

static inline float mavlink_synchropter_sim_data_get_vel_north(const uint8_t* data)
{
	return FloatParse(data, 48);
}

static inline float mavlink_synchropter_sim_data_get_vel_east(const uint8_t* data)
{
	return FloatParse(data, 52);
}

static inline float mavlink_synchropter_sim_data_get_vel_down(const uint8_t* data)
{
	return FloatParse(data, 56);
}

static inline int32_t mavlink_synchropter_sim_data_get_lat(const uint8_t* data)
{
	return Int32Parse(data, 60);
}

static inline int32_t mavlink_synchropter_sim_data_get_lon(const uint8_t* data)
{
	return Int32Parse(data, 64);
}

static inline float mavlink_synchropter_sim_data_get_alt(const uint8_t* data)
{
	return FloatParse(data, 68);
}

static inline float mavlink_synchropter_sim_data_get_alt_baro(const uint8_t* data)
{
	return FloatParse(data, 72);
}

static inline float mavlink_synchropter_sim_data_get_alt_rate(const uint8_t* data)
{
	return FloatParse(data, 76);
}

static inline float mavlink_synchropter_sim_data_get_temperature(const uint8_t* data)
{
	return FloatParse(data, 80);
}

static inline short mavlink_synchropter_sim_data_get_cas(const uint8_t* data)
{
	return ShortParse(data, 84);
}

static inline short mavlink_synchropter_sim_data_get_tas(const uint8_t* data)
{
	return ShortParse(data, 86);
}

static inline float mavlink_synchropter_sim_data_get_baro_pressure(const uint8_t* data)
{
	return FloatParse(data, 88);
}

static inline float mavlink_synchropter_sim_data_get_alt_agl(const uint8_t* data)
{
	return FloatParse(data, 92);
}

// static inline float mavlink_synchropter_sim_data_get_abs_pressure(const uint8_t* data)
// {
// 	return FloatParse(data, 48);
// }

// static inline uint16_t mavlink_vrdrone_sim_data_get_eph(const uint8_t* data)
// {
// 	return UInt16Parse(data, 78);
// }

// static inline uint16_t mavlink_vrdrone_sim_data_get_epv(const uint8_t* data)
// {
// 	return UInt16Parse(data, 80);
// }

// static inline int8_t mavlink_vrdrone_sim_data_get_battery_remain(const uint8_t* data)
// {
// 	return Int8Parse(data, 82);
// }

static inline void mavlink_synchropter_sim_data_decode(const uint8_t* data, synchropter_sim_data_s* synchropter_sim_data)
{
	synchropter_sim_data->time_usec = mavlink_synchropter_sim_data_get_time_usec(data);

	mavlink_synchropter_sim_data_get_attitude_q(data, synchropter_sim_data->attitude_quaternion);

	PX4_DEBUG(synchropter_sim_data->roll_rate);
	synchropter_sim_data->roll_rate = mavlink_synchropter_sim_data_get_roll_rate(data);
	synchropter_sim_data->pitch_rate = mavlink_synchropter_sim_data_get_pitch_rate(data);
	synchropter_sim_data->yaw_rate = mavlink_synchropter_sim_data_get_yaw_rate(data);


	synchropter_sim_data->accel_x = mavlink_synchropter_sim_data_get_accel_x(data);
	synchropter_sim_data->accel_y = mavlink_synchropter_sim_data_get_accel_y(data);
	synchropter_sim_data->accel_z = mavlink_synchropter_sim_data_get_accel_z(data);

	synchropter_sim_data->vel_north = mavlink_synchropter_sim_data_get_vel_north(data);
	synchropter_sim_data->vel_east = mavlink_synchropter_sim_data_get_vel_east(data);
	synchropter_sim_data->vel_down = mavlink_synchropter_sim_data_get_vel_down(data);


	synchropter_sim_data->lat = mavlink_synchropter_sim_data_get_lat(data);
	synchropter_sim_data->lon = mavlink_synchropter_sim_data_get_lon(data);
	synchropter_sim_data->alt = mavlink_synchropter_sim_data_get_alt(data);

	synchropter_sim_data->alt = mavlink_synchropter_sim_data_get_alt(data);
	synchropter_sim_data->alt_baro = mavlink_synchropter_sim_data_get_alt_baro(data);
	synchropter_sim_data->alt_rate = mavlink_synchropter_sim_data_get_alt_rate(data);

	synchropter_sim_data->temperature = mavlink_synchropter_sim_data_get_temperature(data);
	synchropter_sim_data->cas = mavlink_synchropter_sim_data_get_cas(data);
	synchropter_sim_data->tas = mavlink_synchropter_sim_data_get_tas(data);

	synchropter_sim_data->baro_pressure = mavlink_synchropter_sim_data_get_baro_pressure(data);
	synchropter_sim_data->alt_agl = mavlink_synchropter_sim_data_get_alt_agl(data);

	// vrdrone_sim_data->eph = mavlink_vrdrone_sim_data_get_eph(data);
	// vrdrone_sim_data->epv = mavlink_vrdrone_sim_data_get_epv(data);
	// vrdrone_sim_data->battery_remain = mavlink_vrdrone_sim_data_get_battery_remain(data);

	return;
}

#pragma once

typedef struct __vrdrone_sim_data {
	uint64_t time_usec;
	float attitude_quaternion[4];
	float xacc;
	float yacc;
	float zacc;
	float xgyro;
	float ygyro;
	float zgyro;
	float abs_pressure;
	float temperature;
	int16_t ind_airspeed;
	int16_t true_airspeed;
	int32_t lat;
	int32_t lon;
	int32_t alt;
	int16_t vn;
	int16_t ve;
	int16_t vd;
	uint16_t eph;
	uint16_t epv;
	int8_t battery_remain;
} vrdrone_sim_data_s;

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

static inline uint64_t mavlink_vrdrone_sim_data_get_time_usec(const uint8_t* data)
{
	return UInt64Parse(data, 0);
}

static inline void mavlink_vrdrone_sim_data_get_attitude_q(const uint8_t* data, float* attitude_q)
{
	FloatArrayParse(data, attitude_q, sizeof(float) * 4, 8);
}

static inline float mavlink_vrdrone_sim_data_get_xacc(const uint8_t* data)
{
	return FloatParse(data, 24);
}

static inline float mavlink_vrdrone_sim_data_get_yacc(const uint8_t* data)
{
	return FloatParse(data, 28);
}

static inline float mavlink_vrdrone_sim_data_get_zacc(const uint8_t* data)
{
	return FloatParse(data, 32);
}

static inline float mavlink_vrdrone_sim_data_get_xgyro(const uint8_t* data)
{
	return FloatParse(data, 36);
}

static inline float mavlink_vrdrone_sim_data_get_ygyro(const uint8_t* data)
{
	return FloatParse(data, 40);
}

static inline float mavlink_vrdrone_sim_data_get_zgyro(const uint8_t* data)
{
	return FloatParse(data, 44);
}

static inline float mavlink_vrdrone_sim_data_get_abs_pressure(const uint8_t* data)
{
	return FloatParse(data, 48);
}

static inline float mavlink_vrdrone_sim_data_get_temperature(const uint8_t* data)
{
	return FloatParse(data, 52);
}

static inline int16_t mavlink_vrdrone_sim_data_get_ind_airspeed(const uint8_t* data)
{
	return Int16Parse(data, 56);
}

static inline int16_t mavlink_vrdrone_sim_data_get_true_airspeed(const uint8_t* data)
{
	return Int16Parse(data, 58);
}

static inline int32_t mavlink_vrdrone_sim_data_get_lat(const uint8_t* data)
{
	return Int32Parse(data, 60);
}

static inline int32_t mavlink_vrdrone_sim_data_get_lon(const uint8_t* data)
{
	return Int32Parse(data, 64);
}

static inline int32_t mavlink_vrdrone_sim_data_get_alt(const uint8_t* data)
{
	return Int32Parse(data, 68);
}

static inline int16_t mavlink_vrdrone_sim_data_get_vn(const uint8_t* data)
{
	return Int16Parse(data, 72);
}

static inline int16_t mavlink_vrdrone_sim_data_get_ve(const uint8_t* data)
{
	return Int16Parse(data, 74);
}

static inline int16_t mavlink_vrdrone_sim_data_get_vd(const uint8_t* data)
{
	return Int16Parse(data, 76);
}

static inline uint16_t mavlink_vrdrone_sim_data_get_eph(const uint8_t* data)
{
	return UInt16Parse(data, 78);
}

static inline uint16_t mavlink_vrdrone_sim_data_get_epv(const uint8_t* data)
{
	return UInt16Parse(data, 80);
}

static inline int8_t mavlink_vrdrone_sim_data_get_battery_remain(const uint8_t* data)
{
	return Int8Parse(data, 82);
}

static inline void mavlink_vrdrone_sim_data_decode(const uint8_t* data, vrdrone_sim_data_s* vrdrone_sim_data)
{
	vrdrone_sim_data->time_usec = mavlink_vrdrone_sim_data_get_time_usec(data);
	mavlink_vrdrone_sim_data_get_attitude_q(data, vrdrone_sim_data->attitude_quaternion);
	vrdrone_sim_data->xacc = mavlink_vrdrone_sim_data_get_xacc(data);
	vrdrone_sim_data->yacc = mavlink_vrdrone_sim_data_get_yacc(data);
	vrdrone_sim_data->zacc = mavlink_vrdrone_sim_data_get_zacc(data);
	vrdrone_sim_data->xgyro = mavlink_vrdrone_sim_data_get_xgyro(data);
	vrdrone_sim_data->ygyro = mavlink_vrdrone_sim_data_get_ygyro(data);
	vrdrone_sim_data->zgyro = mavlink_vrdrone_sim_data_get_zgyro(data);
	vrdrone_sim_data->abs_pressure = mavlink_vrdrone_sim_data_get_abs_pressure(data);
	vrdrone_sim_data->temperature = mavlink_vrdrone_sim_data_get_temperature(data);
	vrdrone_sim_data->ind_airspeed = mavlink_vrdrone_sim_data_get_ind_airspeed(data);
	vrdrone_sim_data->true_airspeed = mavlink_vrdrone_sim_data_get_true_airspeed(data);
	vrdrone_sim_data->lat = mavlink_vrdrone_sim_data_get_lat(data);
	vrdrone_sim_data->lon = mavlink_vrdrone_sim_data_get_lon(data);
	vrdrone_sim_data->alt = mavlink_vrdrone_sim_data_get_alt(data);
	vrdrone_sim_data->vn = mavlink_vrdrone_sim_data_get_vn(data);
	vrdrone_sim_data->ve = mavlink_vrdrone_sim_data_get_ve(data);
	vrdrone_sim_data->vd = mavlink_vrdrone_sim_data_get_vd(data);
	vrdrone_sim_data->eph = mavlink_vrdrone_sim_data_get_eph(data);
	vrdrone_sim_data->epv = mavlink_vrdrone_sim_data_get_epv(data);
	vrdrone_sim_data->battery_remain = mavlink_vrdrone_sim_data_get_battery_remain(data);

	return;
}

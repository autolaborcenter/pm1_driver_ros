//
// Created by ydrml on 2019/3/15.
//

#ifndef PM1_SDK_MODEL_H
#define PM1_SDK_MODEL_H

#include "chassis_config_t.h"

struct wheels { float left, right; };     // 两轮线速度（m/s）
struct velocity { float v, w; };          // 速度空间（标准单位）
struct physical { float speed, rudder; }; // 物理模型

struct wheels physical_to_wheels(
		const struct physical *,
		const struct chassis_config_t *);

struct physical wheels_to_physical(
		const struct wheels *,
		const struct chassis_config_t *);

struct velocity physical_to_velocity(
		const struct physical *,
		const struct chassis_config_t *);

struct physical velocity_to_physical(
		const struct velocity *,
		const struct chassis_config_t *);

struct wheels velocity_to_wheels(
		const struct velocity *,
		const struct chassis_config_t *);

struct velocity wheels_to_velocity(
		const struct wheels *,
		const struct chassis_config_t *);


#endif //PM1_SDK_MODEL_H

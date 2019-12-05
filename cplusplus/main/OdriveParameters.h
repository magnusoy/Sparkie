#ifndef _ODRIVEPARAMETERS_H_
#define _ODRIVEPARAMETERS_H_

/** Odrive limitations */
#define MOTOR_SPEED_LIMIT 40000.0f
#define MOTOR_CURRENT_LIMIT 40.0f

/** Speed increase multiplier */
#define MOTOR_SPEED_MULTIPLIER 30
#define MOTOR_SPEED_LOWER 0
#define MOTOR_SPEED_UPPER 2000

/** Motor structure */
typedef struct {
    const int INNER = 0;
    const int OUTER = 1;
} Motor;

#endif // _ODRIVEPARAMETERS_H_
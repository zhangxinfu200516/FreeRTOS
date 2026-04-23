#include "crt_force_control_chassis.h"
//x方向
KalmanFilter_t vaEstimateKF;	   // 卡尔曼滤波器结构体
QEKF_INS_t vaEstimateQEKF;		   // 增量式卡尔曼滤波器结构体
//y方向
KalmanFilter_t vaEstimateKF_Y;	   // 卡尔曼滤波器结构体
QEKF_INS_t vaEstimateQEKF_Y;		   // 增量式卡尔曼滤波器结构体

float vaEstimateKF_F[4] = {1.0f, 0.001f, 
                           0.0f, 1.0f};	   // 状态转移矩阵，控制周期为0.002s

float vaEstimateKF_P[4] = {1.0f, 0.0f,
                           0.0f, 1.0f};    // 后验估计协方差初始值

float vaEstimateKF_Q[4] = {0.5f, 0.0f, 
                           0.0f, 200.0f};    // Q矩阵初始值

float vaEstimateKF_R[4] = {1000.0f, 0.0f, 
                            0.0f,  0.001f}; 	
// float vaEstimateKF_Q[4] = {0.5f, 0.0f, 
//                            0.0f, 200.0f};    
// // Q矩阵初始值     Q11减小更信任用加速度估计出来的速度，Q22增大，对加速度的滤波效果减小
// //R11 增大R11减小对轮速的信任度，但是要足够大，太大了会造成转向结束震荡（因为转向时速度实际上应该更信任轮子，实际上是速度滞后太大了不收敛到真实的速度
// //在不打滑的前提下，轮速应该是更准确的，所以在没问题的前提下应该更信任轮子
// float vaEstimateKF_R[4] = {1000.0f, 0.0f, 
//                             0.0f,  0.001f}; 	//v a观测噪声														
float vaEstimateKF_K[4];
													 
const float vaEstimateKF_H[4] = {1.0f, 0.0f,
                                 0.0f, 1.0f};	// 设置矩阵H为常量
float vel_acc_X[2];
float vel_acc_Y[2];
/* Private types -------------------------------------------------------------*/

void xvEstimateKF_Init(KalmanFilter_t *EstimateKF)
{
    Kalman_Filter_Init(EstimateKF, 2, 0, 2);	// 状态向量2维 没有控制量 测量向量2维
	
	memcpy(EstimateKF->F_data, vaEstimateKF_F, sizeof(vaEstimateKF_F));
    memcpy(EstimateKF->P_data, vaEstimateKF_P, sizeof(vaEstimateKF_P));
    memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
    memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
    memcpy(EstimateKF->H_data, vaEstimateKF_H, sizeof(vaEstimateKF_H));
}

void xvEstimateKF_Update(KalmanFilter_t *EstimateKF ,float acc,float vel,float *data)
{   	
	 memcpy(EstimateKF->Q_data, vaEstimateKF_Q, sizeof(vaEstimateKF_Q));
   memcpy(EstimateKF->R_data, vaEstimateKF_R, sizeof(vaEstimateKF_R));
	
    //卡尔曼滤波器测量值更新
    EstimateKF->MeasuredVector[0] =	vel;//测量速度
    EstimateKF->MeasuredVector[1] = acc;//测量加速度
    		
    //卡尔曼滤波器更新函数
    Kalman_Filter_Update(EstimateKF,&vaEstimateQEKF);

    // 提取估计值
    for (uint8_t i = 0; i < 2; i++)
    {
      data[i] = EstimateKF->FilteredValue[i];
    }
}

void Class_Chassis::Init()
{
    // PID初始化

    // 底盘速度xPID, 输出摩擦力
    PID_Velocity_X.Init(200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.002f);

    // 底盘速度yPID, 输出摩擦力
    PID_Velocity_Y.Init(200.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1000.0f, 0.002f);

    // 底盘角速度PID, 输出扭矩
    PID_Omega.Init(20.0f, 0.0f, 0.0f, 0.0f, 0.0f, 100.0f, 0.002f);

    // 轮向电机ID初始化
   // 电机初始化
    Motor_Wheel[0].Init(&hcan1, Motor_DJI_ID_0x201, Motor_DJI_Control_Method_CURRENT, 13.933f, Motor_DJI_Power_Limit_Status_ENABLE);
    Motor_Wheel[0].PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    // 电机初始化
    Motor_Wheel[1].Init(&hcan1, Motor_DJI_ID_0x204, Motor_DJI_Control_Method_CURRENT, 13.933f, Motor_DJI_Power_Limit_Status_ENABLE);
    Motor_Wheel[1].PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    // 电机初始化
    Motor_Wheel[2].Init(&hcan1, Motor_DJI_ID_0x203, Motor_DJI_Control_Method_CURRENT, 13.933f, Motor_DJI_Power_Limit_Status_ENABLE);
    Motor_Wheel[2].PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);
    // 电机初始化
    Motor_Wheel[3].Init(&hcan1, Motor_DJI_ID_0x202, Motor_DJI_Control_Method_CURRENT, 13.933f, Motor_DJI_Power_Limit_Status_ENABLE);
    Motor_Wheel[3].PID_Omega.Init(1.0f, 0.0f, 0.0f, 0.0f, 20.0f, 20.0f);

    // 超级电容初始化
    Supercap.Init(&hcan1, 75);

    //imu初始化
    Boardc_BMI.Init();
    //低通滤波初始化
    //实际速度滤波
    PID_Velocity_Filter[0].Init(-4.0f,4.0f,Filter_Fourier_Type_LOWPASS,10.0f,0.0f,500.0f,5);
    PID_Velocity_Filter[1].Init(-4.0f,4.0f,Filter_Fourier_Type_LOWPASS,10.0f,0.0f,500.0f,5);
    PID_Omega_Filter.Init(-20.0f,20.0f,Filter_Fourier_Type_LOWPASS,10.0f,0.0f,500.0f,15);


    xvEstimateKF_Init(&vaEstimateKF);
    
    xvEstimateKF_Init(&vaEstimateKF_Y);

   // 斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.064f, 0.064f);
    // 斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.064f, 0.064f);
    // 斜坡函数加减速角速度
    Slope_Omega.Init(0.1f, 0.1f);
    //kalman滤波器初始化
    kalman_Init(&kalman_MotionAccel_nx,0.9,0.005,0,1);
    kalman_Init(&kalman_MotionAccel_ny,0.9,0.005,0,1);
    kalman_Init(&kalman_Now_VelocityX,0.9,0.005,0,1);
    kalman_Init(&kalman_Now_VelocityY,0.9,0.005,0,1);
    kalman_Init(&kalman_Now_VelocityZ,0.9,0.005,0,1);
}

//#define Control_Type_Oemga
#define Control_Type_Current

/*
1.运动学逆解算
2.
*/
void Class_Chassis::TIM_100ms_Alive_PeriodElapsedCallback()
{
    for (int i = 0; i < 4; i++)
    {
       // Motor_Steer[i].TIM_100ms_Alive_PeriodElapsedCallback();
        Motor_Wheel[i].TIM_100ms_Alive_PeriodElapsedCallback();
    }
}
/**
 * @brief TIM定时器中断解算回调函数
 *
 */
void Class_Chassis::TIM_2ms_Resolution_PeriodElapsedCallback()
{

    Self_Resolution();
    
    PID_Velocity_Filter[0].Set_Now(Now_Velocity_X);
    PID_Velocity_Filter[0].TIM_Adjust_PeriodElapsedCallback(0.00f);
    PID_Velocity_Filter[1].Set_Now(Now_Velocity_Y);
    PID_Velocity_Filter[1].TIM_Adjust_PeriodElapsedCallback(0.0f);
    PID_Omega_Filter.Set_Now(Now_Omega);
    PID_Omega_Filter.TIM_Adjust_PeriodElapsedCallback(0.00f);


    // 斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Target(Target_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Omega.Set_Target(Target_Omega);
    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
    //卡尔曼滤波器更新车体x方向速度
    //xvEstimateKF_Update(&vaEstimateKF,Acceleration_X_Filter.Get_Out(),Now_Velocity_X);
    
    //INS_Data.Mix_Velocity_X = vel_acc[0];


}

void Class_Chassis::TIM_1ms_Kalmancale_PeriodElapsedCallback()
{
    kalman_set_now(&kalman_MotionAccel_nx,Boardc_BMI.Get_MotionAccel_b_x());
    Recv_Adjust_PeriodElapsedCallback(&kalman_MotionAccel_nx);
    kalman_set_now(&kalman_MotionAccel_ny,Boardc_BMI.Get_MotionAccel_b_y());
    Recv_Adjust_PeriodElapsedCallback(&kalman_MotionAccel_ny);
    kalman_set_now(&kalman_Now_VelocityX,Now_Velocity_X);
    Recv_Adjust_PeriodElapsedCallback(&kalman_Now_VelocityX);
    kalman_set_now(&kalman_Now_VelocityY,Now_Velocity_Y);
    Recv_Adjust_PeriodElapsedCallback(&kalman_Now_VelocityY);
    kalman_set_now(&kalman_Now_VelocityZ,Now_Omega);
    Recv_Adjust_PeriodElapsedCallback(&kalman_Now_VelocityZ);
    //卡尔曼滤波器更新车体x方向速度
    //1.加速度转换到机体系下
    //2.
    xvEstimateKF_Update(&vaEstimateKF,kalman_MotionAccel_nx.Out,kalman_Now_VelocityX.Out,vel_acc_X);
    xvEstimateKF_Update(&vaEstimateKF_Y,kalman_MotionAccel_ny.Out,kalman_Now_VelocityY.Out,vel_acc_Y);
}
/**
 * @brief TIM定时器中断控制回调函数
 *
 */
void Class_Chassis::TIM_2ms_Control_PeriodElapsedCallback()
{
    Kinematics_Inverse_Resolution();

    #ifdef Control_Type_Current

    Output_To_Dynamics(); 

    Dynamics_Inverse_Resolution();

    #endif

    Output_To_Motor();
}


/**
 * @brief 运动学逆解算
 *
 */
void Class_Chassis::Kinematics_Inverse_Resolution()
{
    // for (int i = 0; i < 4; i++)
    // {
    //     float tmp_velocity_x, tmp_velocity_y, tmp_velocity_modulus;

    //     // 解算到每个轮组的具体线速度
    //     tmp_velocity_x = Target_Velocity_X - Target_Omega * Wheel_To_Core_Distance[i] * arm_sin_f32(Wheel_Azimuth[i]);
    //     tmp_velocity_y = Target_Velocity_Y + Target_Omega * Wheel_To_Core_Distance[i] * arm_cos_f32(Wheel_Azimuth[i]);
    //     arm_sqrt_f32(tmp_velocity_x * tmp_velocity_x + tmp_velocity_y * tmp_velocity_y, &tmp_velocity_modulus);

    //     // 根据线速度决定轮向电机角速度
    //     Target_Wheel_Omega[i] = tmp_velocity_modulus / Wheel_Radius;
    // }
    float motor0_temp_linear_vel = -1.0f * (arm_cos_f32(PI / 4.0f) * Target_Velocity_Y - arm_cos_f32(PI / 4.0f) * Target_Velocity_X + Target_Omega * Wheel_To_Core_Distance);
    float motor1_temp_linear_vel = -1.0f * (-arm_cos_f32(PI / 4.0f) * Target_Velocity_Y - arm_cos_f32(PI / 4.0f) * Target_Velocity_X + Target_Omega * Wheel_To_Core_Distance);
    float motor2_temp_linear_vel = -1.0f * (-arm_cos_f32(PI / 4.0f) * Target_Velocity_Y + arm_cos_f32(PI / 4.0f) * Target_Velocity_X + Target_Omega * Wheel_To_Core_Distance);
    float motor3_temp_linear_vel = -1.0f * (arm_cos_f32(PI / 4.0f) * Target_Velocity_Y + arm_cos_f32(PI / 4.0f) * Target_Velocity_X + Target_Omega * Wheel_To_Core_Distance);
    Target_Wheel_Omega[0] = motor0_temp_linear_vel / Wheel_Radius;
    Target_Wheel_Omega[1] = motor1_temp_linear_vel / Wheel_Radius;
    Target_Wheel_Omega[2] = motor2_temp_linear_vel / Wheel_Radius;
    Target_Wheel_Omega[3] = motor3_temp_linear_vel / Wheel_Radius;
    
    // _Steer_Motor_Kinematics_Nearest_Transposition();
}
/**
 * @brief 输出到动力学状态
 *
 */
void Class_Chassis::Output_To_Dynamics()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE__):
    {
        // 底盘失能
        for (int i = 0; i < 4; i++)
        {
            PID_Velocity_X.Set_Integral_Error(0.0f);
            PID_Velocity_Y.Set_Integral_Error(0.0f);
            PID_Omega.Set_Integral_Error(0.0f);
        }

        break;
    }
    case (Chassis_Control_Type_FLLOW__):
    case (Chassis_Control_Type_SPIN__):
    case (Chassis_Control_Type_ANTI_SPIN__):
    {
        //小陀螺模式下以不用融合的速度：测试效果为底盘偏心转
        PID_Velocity_X.Set_Target(Slope_Velocity_X.Get_Out());
        PID_Velocity_Y.Set_Target(Slope_Velocity_Y.Get_Out());
        PID_Omega.Set_Target(Slope_Omega.Get_Out());

        if(Chassis_Control_Type == Chassis_Control_Type_FLLOW__)
        {
            PID_Velocity_X.Set_Now(kalman_Now_VelocityX.Out);
            PID_Velocity_Y.Set_Now(kalman_Now_VelocityY.Out);
        }
        else
        {
            PID_Velocity_X.Set_Now(kalman_Now_VelocityX.Out);
            PID_Velocity_Y.Set_Now(kalman_Now_VelocityY.Out);
        }
        PID_Omega.Set_Now(kalman_Now_VelocityZ.Out);
    
        PID_Velocity_X.TIM_Adjust_PeriodElapsedCallback();
        PID_Velocity_Y.TIM_Adjust_PeriodElapsedCallback();
        PID_Omega.TIM_Adjust_PeriodElapsedCallback();

        break;
    }
    }
}
/**
 * @brief 动力学逆解算
 *
 */
void Class_Chassis::Dynamics_Inverse_Resolution()
{
    float force_x, force_y, torque_omega;

    force_x = PID_Velocity_X.Get_Out();
    force_y = PID_Velocity_Y.Get_Out();
    torque_omega = PID_Omega.Get_Out();

    float sqrt2_4 = 0.0f;
    arm_sqrt_f32(1.0f / 8.0f, &sqrt2_4);
    // 每个轮的扭力
    // 解算到每个轮组的具体摩擦力
    float tmp_force[4];
    tmp_force[0] = sqrt2_4 * (-force_x + force_y) + torque_omega / (4.0f * Wheel_To_Core_Distance);
    tmp_force[1] = sqrt2_4 * (-force_x - force_y) + torque_omega / (4.0f * Wheel_To_Core_Distance);
    tmp_force[2] = sqrt2_4 * (force_x - force_y) + torque_omega / (4.0f * Wheel_To_Core_Distance);
    tmp_force[3] = sqrt2_4 * (force_x + force_y) + torque_omega / (4.0f * Wheel_To_Core_Distance);
    
    for (int i = 0; i < 4; i++)
    {
        //#define force (20.0f/(0.154f/2.0f)*7.0f*0.01562f) 预测最大的摩擦力为28.4N左右
        // 摩擦力转换至扭矩 
        Target_Wheel_Current[i] = -1.0f * (tmp_force[i] * Wheel_Radius) + Wheel_Speed_Limit_Factor * (Target_Wheel_Omega[i] - Motor_Wheel[i].Get_Now_Omega()) ;//(tmp_force[i] * Wheel_Radius) / (13.933f * 0.5f) / M3508_Kt;//Wheel_Speed_Limit_Factor * (Target_Wheel_Omega[i] - Motor_Wheel[i].Get_Now_Omega());
        // 动摩擦阻力前馈
        if (Target_Wheel_Omega[i] > Wheel_Resistance_Omega_Threshold)
        {
            Target_Wheel_Current[i] += Dynamic_Resistance_Wheel_Current[i];
        }
        else if (Target_Wheel_Omega[i] < -Wheel_Resistance_Omega_Threshold)
        {
            Target_Wheel_Current[i] -= Dynamic_Resistance_Wheel_Current[i];
        }
        else
        {
            Target_Wheel_Current[i] += Motor_Wheel[i].Get_Now_Omega() / Wheel_Resistance_Omega_Threshold * Dynamic_Resistance_Wheel_Current[i];
        }
    }

    // 根据斜坡与压力进行电流限幅防止贴地打滑
    // TODO
}

/**
 * @brief 输出到电机
 *
 */
void Class_Chassis::Output_To_Motor()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE__):
    {
        // 底盘失能
        for (int i = 0; i < 4; i++)
        {
            
            Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_CURRENT);

            Motor_Wheel[i].Set_Target_Current(0.0f);
        }

        break;
    }
    case (Chassis_Control_Type_FLLOW__):
    case (Chassis_Control_Type_SPIN__):
    case (Chassis_Control_Type_ANTI_SPIN__):
    {   
        #ifdef Control_Type_Current
        // 全向模型
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
        }

        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_Target_Current(Target_Wheel_Current[i]);
        }
        #endif
        #ifdef Control_Type_Oemga
        for (int i = 0; i < 4; i++)
        {
            Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
            Motor_Wheel[i].Set_Target_Omega(Target_Wheel_Omega[i]);
        }
        #endif

        break;
    }
    }

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].TIM_Calculate_PeriodElapsedCallback();
    }

    // //进行功率限制
    // if(Referee->Get_Referee_Status() == Referee_Status_ENABLE)
    // {
    //     float energyBuffer = Referee->Get_Chassis_Energy_Buffer();
    //     // 归一化到[-1, 1]范围，中心点在30J
    //     float normalized = (energyBuffer - 30.0f) / 30.0f;
    //     // 使用tanh实现平滑过渡，范围[-30, 30]
    //     float bufferPower = 30.0f * tanhf(normalized);
        
    //     Power_Management.Max_Power = Referee->Get_Chassis_Power_Max() + bufferPower;
    // }
    // else
    // {
    //     Power_Management.Max_Power = 100.0f;
    // }

    Power_Management.Max_Power = Supercap.Get_Chassis_Device_LimitPower();

    Power_Management.Total_error = 0.0;
    Power_Limit.Power_Task(Power_Management);

    for (int i = 0; i < 4; i++)
    {
        Motor_Wheel[i].Reset_Set_Out_And_Output(Power_Management.Motor_Data[i].output);
    }
}

/**
 * @brief 自身解算
 *
 */
#define filter_complementary
uint32_t imu_cnt_1 = 0,imu_cnt_2 = 0;
float alpha_v_adaptive = 0.9f,alpha_omega_adaptive = 0.0f;
void Class_Chassis::Self_Resolution()
{
    // 根据电机编码器与陀螺仪计算速度和角度

    Now_Velocity_X = 0.0f;
    Now_Velocity_Y = 0.0f;
    Now_Omega = 0.0f;

    // 轮速计的计算方式

    float wheel_vel0 = Motor_Wheel[0].Get_Now_Omega() * Wheel_Radius;
    float wheel_vel1 = Motor_Wheel[1].Get_Now_Omega() * Wheel_Radius;
    float wheel_vel2 = Motor_Wheel[2].Get_Now_Omega() * Wheel_Radius;
    float wheel_vel3 = Motor_Wheel[3].Get_Now_Omega() * Wheel_Radius;

    // 从逆运动学方程推导正运动学：

    // 建立轮速向量
    float wheel_vel[4] = {wheel_vel0, wheel_vel1, wheel_vel2, wheel_vel3};
    float K = 0.0f;
    arm_sqrt_f32(0.5f, &K);
    // 因此正运动学公式为：
    Now_Velocity_X = -1.0f * (-wheel_vel0 - wheel_vel1 + wheel_vel2 + wheel_vel3) / (4 * K);
    Now_Velocity_Y = -1.0f * (wheel_vel0 - wheel_vel1 - wheel_vel2 + wheel_vel3) / (4 * K);
    Now_Omega = -1.0f *(wheel_vel0 + wheel_vel1 + wheel_vel2 + wheel_vel3) / (4 * Wheel_To_Core_Distance);


    // 角度解算
    // float pitch = -AHRS_Chassis->Get_Angle_Pitch();
    // float roll = AHRS_Chassis->Get_Angle_Roll();
    // if (isnan(pitch) == true || isnan(roll) == true)
    // {
    //     Angle_Pitch = 0.0f;
    //     Angle_Roll = 0.0f;
    // }
    // else
    // {
    //     Angle_Pitch = pitch;
    //     Angle_Roll = roll;
    // }

    // Slope_Direction_X = arm_sin_f32(pitch) * arm_cos_f32(roll);
    // Slope_Direction_Y = -arm_sin_f32(roll);
    // Slope_Direction_Z = arm_cos_f32(pitch) * arm_cos_f32(roll);
    //转换电机数据到Power_Management的量纲
    for (int i = 0; i < 4; i++)         //数据传递处理
    {
        //都是计算转子的
        Power_Management.Motor_Data[i].feedback_omega = Motor_Wheel[i].Get_Now_Omega() /  RPM_TO_RADPS *  13.933f;
        Power_Management.Motor_Data[i].feedback_torque = Motor_Wheel[i].Get_Now_Current() * 16384.0f / 20.0f * M3508_CMD_CURRENT_TO_TORQUE;     //与减速比有关
        Power_Management.Motor_Data[i].torque = Motor_Wheel[i].Get_Target_Current() * 16384.0f / 20.0f * M3508_CMD_CURRENT_TO_TORQUE;                     //与减速比有关
        Power_Management.Motor_Data[i].pid_output = Motor_Wheel[i].Get_Target_Current() * 16384.0f / 20.0f;

        //Power_Management.Motor_Data[i].Target_error = fabs(Motor_Wheel[i].Get_Target_Omega_Radian() - Motor_Wheel[i].Get_Now_Omega_Radian());
        
    }

}

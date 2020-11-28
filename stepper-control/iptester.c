#include "iptester.h"

// void max_accel_process(void)
// {
//     accel_stable_check();
//     if (accel_stable_flag)
//     {
//         // NRF_LOG_RAW_INFO("it worked");
//         for(uint8_t i = 0; i < 3; i++)
//         {
//             if (accel_data_avg[i] > accel_data_max[i])
//             {
//                 accel_data_max[i] = accel_data_avg[i];
//                 update_offset_flag = true;
//             }

//             if (accel_data_avg[i] < accel_data_min[i])
//             {
//                 accel_data_min[i] = accel_data_avg[i];
//                 update_offset_flag = true;
//             }
//         }
//         // accel_stable_flag = false;
//     }
// }

// void accel_stable_process(int16_t *raw_acc_data)
// {
//     if (accel_channel_count > 0)
//     {
//             if (raw_acc_data[i] > accel_inst_max[i])
//             {
//                 accel_inst_max[i] = raw_acc_data[i];
//             }
//             if (raw_acc_data[i] < accel_inst_min[i])
//             {
//                 accel_inst_min[i] = raw_acc_data[i];
//             }
//     } else
//     {
//             accel_inst_max[i] = raw_acc_data[i];
//             accel_inst_min[i] = raw_acc_data[i];
//     }
// }

// void accel_stable_check(void)
// {
//     accel_stable_flag = true;
//     for(uint8_t i = 0; i < 3; i++)
//     {
//         if((accel_inst_max[i]-accel_inst_min[i]) > ACCEL_STABLE)
//         {
//             accel_stable_flag = false;
//         }
//     }
// }
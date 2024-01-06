#include "ikarashiCAN_mk2.h"

CANFilter::CANFilter(PinName rd, PinName td, int frequency) : CAN(rd, td, frequency)
{
}

int CANFilter::my_filter(int mode, uint32_t id_high, uint32_t id_low, uint32_t mask_high, uint32_t mask_low, int32_t handle)
{
    lock();
    id_low = 0;
    mask_low = 0;
    int ret = 0;
    ret = this->mycan_filter(mode, &_can, id_high, id_low, mask_high, mask_low, CANStandard, handle);
    unlock();
    return ret;
}

#ifdef FDCAN1
int CANFilter::mycan_filter(int mode, can_t *obj, uint32_t id_high, uint32_t id_low, uint32_t mask_high, uint32_t mask_low, CANFormat format, int32_t handle)
{
    UNUSED(handle); // Not supported yet (seems to be a used in read function?)

    FDCAN_FilterTypeDef sFilterConfig = {0};

    if (format == CANStandard)
    {
        sFilterConfig.IdType = FDCAN_STANDARD_ID;
        sFilterConfig.FilterIndex = 0;
        sFilterConfig.FilterType = FDCAN_FILTER_MASK;
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        sFilterConfig.FilterID1 = id_high;
        sFilterConfig.FilterID2 = mask_high;
    }
    else if (format == CANExtended)
    {
        sFilterConfig.IdType = FDCAN_EXTENDED_ID;
        sFilterConfig.FilterIndex = 0;
        sFilterConfig.FilterType = FDCAN_FILTER_MASK;
        sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        sFilterConfig.FilterID1 = id_high;
        sFilterConfig.FilterID2 = mask_high;
    }
    else
    { // Filter for CANAny format cannot be configured for STM32
        return 0;
    }

    if (HAL_FDCAN_ConfigFilter(&obj->CanHandle, &sFilterConfig) != HAL_OK)
    {
        return 0;
    }

    return 1;
}
#else
int CANFilter::mycan_filter(int mode, can_t *obj, uint32_t id_high, uint32_t id_low, uint32_t mask_high, uint32_t mask_low, CANFormat format, int32_t handle)
{
    int success = 0;

    // filter for CANAny format cannot be configured for STM32
    if ((format == CANStandard) || (format == CANExtended))
    {
        CAN_FilterConfTypeDef sFilterConfig;
        sFilterConfig.FilterNumber = handle;
        if (mode)
            sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
        else
            sFilterConfig.FilterMode = CAN_FILTERMODE_IDLIST;
        sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        // sFilterConfig.FilterScale = CAN_FILTERSCALE_16BIT;

        if (format == CANStandard)
        {
            sFilterConfig.FilterIdHigh = id_high << 5;
            sFilterConfig.FilterIdLow = id_low;
            sFilterConfig.FilterMaskIdHigh = mask_high << 5;
            sFilterConfig.FilterMaskIdLow = mask_low; // allows both remote and data frames
        }
        else
        {                                                                     // format == CANExtended
            sFilterConfig.FilterIdHigh = id_high >> 13;                       // EXTID[28:13]
            sFilterConfig.FilterIdLow = (0xFFFF & (id_high << 3)) | (1 << 2); // EXTID[12:0] + IDE
            sFilterConfig.FilterMaskIdHigh = mask_high >> 13;
            sFilterConfig.FilterMaskIdLow = (0xFFFF & (mask_high << 3)) | (1 << 2);
        }

        // sFilterConfig.FilterFIFOAssignment = 0;
        sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        sFilterConfig.FilterActivation = ENABLE;
        sFilterConfig.BankNumber = 14 + handle;

        if (HAL_CAN_ConfigFilter(&obj->CanHandle, &sFilterConfig) == HAL_OK)
        {
            success = 1;
        }
    }

    return success;
}
#endif

ikarashiCAN_mk2::ikarashiCAN_mk2(PinName rd, PinName td, int msgID, int frequency) : can(rd, td, frequency), canfil(rd, td, frequency), no_mutex_can(rd, td, frequency)
{
    this_msgID = msgID;
    sender_len = 0;
    can_flag_send = 0;
    can_flag_read = 0;
}

ikarashiCAN_mk2::~ikarashiCAN_mk2()
{
    /*
    can_flag_send = 0;
    can_flag_read = 0;
    */
}

int ikarashiCAN_mk2::set(uint8_t *_data, size_t size)
{
    sender_len = size;
    memcpy(sender, _data, size);
    return 1;
}

int ikarashiCAN_mk2::set_this_id(uint32_t id)
{
    // this_msgID = id & 0x1f;
    this_msgID = id;
    return (int)this_msgID;
}

int ikarashiCAN_mk2::get(uint8_t *_data, size_t length)
{
    memcpy(_data, msg.data, length);
    return 1;
}

int ikarashiCAN_mk2::get_length()
{
    return (int)msg.len;
}

int ikarashiCAN_mk2::get_id()
{
    return (msg.id & 0x1f);
}

int ikarashiCAN_mk2::get_target_id()
{
    return ((msg.id & 0x3e0) >> 5);
}

int ikarashiCAN_mk2::get_this_id()
{
    return (int)this_msgID;
}

int ikarashiCAN_mk2::get_sender_id()
{
    return msg.id;
}

uint8_t ikarashiCAN_mk2::get_byte(int num)
{
    return msg.data[num];
}

int ikarashiCAN_mk2::get_read_flag()
{
    return can_flag_read;
}

int ikarashiCAN_mk2::get_send_flag()
{
    return can_flag_send;
}

int ikarashiCAN_mk2::write(uint32_t id)
{
    int this_msgID_to_send = ((id & 0x1f) << 5) | this_msgID;
    can_flag_send = can.write(CANMessage(this_msgID_to_send, sender, sender_len));
    return can_flag_send;
}

int ikarashiCAN_mk2::read(int _handle)
{
    can_flag_read = can.read(msg, _handle);
    return can_flag_read;
}

int ikarashiCAN_mk2::filter(int mode, uint32_t id_high, uint32_t mask_high, int32_t handle)
{
    return canfil.my_filter(mode, id_high, 0, mask_high, 0, handle);
}

void ikarashiCAN_mk2::reset()
{
    can.reset();
}

void ikarashiCAN_mk2::read_start()
{
    no_mutex_can.attach(Callback<void()>(this, &ikarashiCAN_mk2::read_for_attach), CAN::RxIrq);
}

void ikarashiCAN_mk2::read_for_attach()
{
    if (can_flag_read = no_mutex_can.read(msg))
    {
        queue.push(msg);
    }
    while (!queue.empty())
    {
        queue.pop(msg);
    }
}
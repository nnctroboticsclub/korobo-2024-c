/**
 * @file ikarashiCAN_mk2.h
 * @author 五十嵐　幸多 (kotakota925ika@gmail.com)
 * @brief いかこうのCANライブラリです。
 * @version 0.1
 * @date 2023-06-29
 */
#include "mbed.h"
#include "NoMutexCAN.h"

#ifndef IKARASHICAN_MK2
#define IKARASHICAN_MK2

/**
 * @brief CANのフィルター用のクラス
 * @note mbedのCANフィルターが使いにくすぎたから、halで実装した。
 *       ぶっちゃけ使わなくていい。ムズイので説明はしません
 *       てか、自分もよくわかってない
 */
class CANFilter : public CAN
{
public:
    CANFilter(PinName rd, PinName td, int frequency = 500000);
    int my_filter(int mode, uint32_t id_high, uint32_t id_low, uint32_t mask_high, uint32_t mask_low, int32_t handle);
    int mycan_filter(int mode, can_t *obj, uint32_t id_high, uint32_t id_low, uint32_t mask_high, uint32_t mask_low, CANFormat format, int32_t handle);

private:
};

/**
 * @brief いかこうのCANクラス（大本命）
 *
 */
class ikarashiCAN_mk2
{
public:
    /**
     * @brief コンストラクタ
     * @param rd canのRDピン
     * @param td canのTDピン
     * @param msgID canのID。範囲は、0~0xff
     * @param frequency 周波数。省略すると自動で500kHzになる
     */
    ikarashiCAN_mk2(PinName rd, PinName td, int msgID, int frequency = 500000);

    /**
     * @brief デストラクタ
     * 
     */
    ~ikarashiCAN_mk2();

    /**
     * @brief 送信するデータをセットする関数
     * 
     * @param _data セットしたいデータ配列
     * @param size セットしたいデータ配列のサイズ
     * @return int 
     */
    int set(uint8_t *_data, size_t size);

    /**
     * @brief CANのIDを変更する
     * 
     * @param id ID
     * @return int 
     */
    int set_this_id(uint32_t id);

    /**
     * @brief 受信したデータをゲットするやつ
     * 
     * @param _data データを入れたい配列
     * @param length 配列のサイズ
     * @return int 
     */
    int get(uint8_t *_data, size_t length);

    /**
     * @brief 1バイトずつデータをゲットする関数。主にデバッグ用
     * 
     * @param num 1〜8 ゲットしたいデータ配列の要素番号を指定する
     * @return uint8_t 受信した1バイトのデータ
     */
    uint8_t get_byte(int num);

    /**
     * @brief 受信したデータのバイト数を返す関数
     * 
     * @return int 受信したデータのバイト数
     */
    int get_length();

    /**
     * @brief 送信相手のIDを取得する関数
     * 
     * @return int データを送信してきたやつのID
     */
    int get_id();

    /**
     * @brief 送信側がwrite関数で指定したIDを取得する関数
     *        これをうまく使うと、特定のIDにデータが送信できる。
     * 
     * @return int 送信側が指定したID
     */
    int get_target_id();

    /**
     * @brief CANのIDを取得する関数
     * 
     * @return int CANのID
     */
    int get_this_id();

    /**
     * @brief 送信側のIDを取得する関数
     * 
     * @return int 送信側のCANIC
     */
    int get_sender_id();

    /**
     * @brief 受信したかどうかの判定用関数
     * 
     * @return int 1:受信した  0:受信してない
     */
    int get_read_flag();

    /**
     * @brief 送信したかどうかの判定用関数
     * 
     * @return int 1:送信した  0:送信してない
     */
    int get_send_flag();

    /**
     * @brief セットしたデータを送信する関数
     * 
     * @param id 送信したいCANのIDを指定する
     * @return int 
     */
    int write(uint32_t id);

    /**
     * @brief データを受信する関数
     * 
     * @param _handle よくわからん。省略していい
     * @return int 
     */
    int read(int _handle = 0);

    /**
     * @brief フィルター関数
     *        実装には苦労したが、使い方がむずいから非推奨
     * 
     * @param mode フィルターの形式を指定する。0 -> IDリスト    1 -> IDマスク
     * @param id_high 
     * @param mask_high 
     * @param handle 
     * @return int 
     */
    int filter(int mode, uint32_t id_high, uint32_t mask_high, int32_t handle);

    /**
     * @brief リセット関数（魔法の呪文）
     * @note なんかうまく通信できないな〜ってときに使うと、なんかうまくいく
     *       割り込み受信中に使うと、エラーが出るみたい
     */
    void reset();

    /**
     * @brief 受信割り込み
     * @note mbedはその特性上、ライブラリ上でcanの受信割り込み処理ができない。だから頑張った
     *       参考:https://qiita.com/hbvcg00/items/7c89f10a00d0c9005870
     * 
     */
    void read_start();

private:
    CAN can;
    CANMessage msg;
    CANFilter canfil;
    NoMutexCAN no_mutex_can;
    CircularBuffer<CANMessage, 32> queue;
    int can_flag_send;
    int can_flag_read;

    int this_msgID;

    /// 送信するデータの配列
    uint8_t sender[64];
    size_t sender_len;

    void read_for_attach();
};

#endif
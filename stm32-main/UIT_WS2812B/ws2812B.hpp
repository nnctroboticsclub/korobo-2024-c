//--------------------------------------------------------
//  SPI を使って WS2812B を点灯するためのクラス（ヘッダ）
//      サポートするボード： Nucleo-F401RE, Nucleo-F446RE
//
//  2016/11/21, Copyright (c) 2016 MIKAMI, Naoki
//--------------------------------------------------------

#include "mbed.h"

#if !defined(STM32F401xE) && !defined(STM32F446xx)
#error Select Nucleo-F401RE or Nucleo-F446RE.
#endif

#ifndef STM32F4xx_WS2812B_HPP
#define STM32F4xx_WS2812B_HPP

namespace Mikami
{
    class WS2812B
    {
    public:
        // コンストラクタ
        //      inv = true:  インバータを介して WS2812B に接続する場合
        //          = false: 直接 WS2812B に接続する場合
        WS2812B(PinName pin, bool inv = false);

        virtual ~WS2812B() {}

        void Write(uint32_t x);         // 一つの LED へ書き込む
        void Write(uint32_t x, int k);  // k 個の LED へ書き込む
        void Reset() { wait_us(50); }
        void Clear(int k);              // k 個の LED を消灯

    private:
        SPI spi_;
        SPI_TypeDef *mySpi_;

        void (WS2812B::*fp)(uint8_t);
        void SendByte(uint8_t x) { (this->*fp)(x); }
        void Send3Bytes(uint8_t x0, uint8_t x1, uint8_t x2);
        void T0HL() { Send3Bytes(0xFE, 0x00, 0x00); }   // 0 を送る
        void T1HL() { Send3Bytes(0xFF, 0xFF, 0x00); }   // 1 を送る
        void SendByteNorm(uint8_t x);   // データをそのまま送る
        void SendByteInv(uint8_t x);    // データを反転して送る

        // コピー･コンストラクタと代入演算子は使用禁止
        WS2812B(const WS2812B&);
        WS2812B& operator=(const WS2812B&);
    };
}
#endif  // STM32F4xx_WS2812B_HPP 

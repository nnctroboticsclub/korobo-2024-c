#include <mbed.h>
#include <ikarashiCAN_mk2.h>
#include <ikako_c620.h>
#include <button_state.h>

#include <robotics/filter/pid.hpp>

using namespace std::chrono_literals;

DigitalOut led1(LED1);
DigitalOut emc(PC_3);
DigitalIn button(PC_13);
Button_state btn_state;
// ikarashiCAN_mk2 ican(PB_5, PB_6, 1000000);
ikarashiCAN_mk2 ican(D15, D14, 0, 1000000);
ikako_c620 ic620[]{
    ikako_c620(1),
};
ikako_c620_sender sender(ic620, 1, &ican);
Thread thread;
Thread thread2;

uint8_t btn;
float speed = 0;

float output = 0;
float effective_output = 0;

struct PIDThread {
  Thread thread;
  robotics::filter::PID<float> pid{1E-4, 1, 0, 0};

  void Impl() {
    while (1) {
      pid.goal_.SetValue(speed);
      pid.fb_.SetValue((uint16_t)ic620[0].get_rpm());

      pid.Update(1E-3);

      output += pid.output_.GetValue() * 1E-3;
      effective_output = std::max(std::min(output * 0.02f, 0.0005f), -0.0005f);
      ThisThread::sleep_for(1ms);
    }
  }

  void Start() { thread.start(callback(this, &PIDThread::Impl)); }
};

PIDThread *pid_thread;
void PIDThread_Start() {
  pid_thread = new PIDThread();
  pid_thread->Start();
}

void main_update() {
  btn = button;
  speed = (btn_state.count(btn, 2)) ? 5 : -5;

  ic620[0].set(effective_output);

  sender.read();
  led1 = ic620[0].get_read_flag();
}

void can_thread() {
  while (1) {
    sender.write();
    ThisThread::sleep_for(1ms);
  }
}

void print_thread() {
  while (1) {
    printf("f: %d%d%d, ", ican.get_read_flag(), ican.get_send_flag(),
           ic620[0].get_read_flag());
    printf("p-gfo: [%8.6f, %8.6f, %8.6f], ", pid_thread->pid.goal_.GetValue(),
           pid_thread->pid.fb_.GetValue(), pid_thread->pid.output_.GetValue());
    printf("o-oe: [%8.6f, %8.6f]", output, effective_output);
    printf("\n");
    ThisThread::sleep_for(10ms);
  }
}

int main() {
  led1 = 0;

  ican.read_start();
  thread.start(&print_thread);
  thread2.start(&can_thread);

  PIDThread_Start();

  emc = 1;

  while (1) {
    main_update();
  }
}
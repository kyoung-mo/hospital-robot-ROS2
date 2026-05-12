#include <micro_ros_arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/string.h>

// 방 번호 설정 (1 또는 2)
// 보드 업로드 전 변경
#define ROOM_ID 1

// 핀 설정
#define PIN_BTN       7
#define PIN_LED_GREEN 2
#define PIN_LED_BLUE  3
#define PIN_LED_RED   4
#define PIN_BUZZER    5

// LCD 설정 (I2C 주소 0x27)
LiquidCrystal_I2C lcd(0x27, 16, 2);

// micro-ROS 설정
rcl_publisher_t publisher_call;
rcl_subscription_t subscriber_event;
std_msgs__msg__String pub_msg;
std_msgs__msg__String sub_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#if ROOM_ID == 1
  #define ROOM_STR  "Room 101"
  #define NODE_NAME "button_led_node_1"
  #define TOPIC_PUB "/hospital/call/room1"
  #define TOPIC_SUB "/hospital/emergency_event/room1"
  // [v6.4] 발행 data: "101" (방 번호 숫자 통일)
  #define ROOM_DATA "101"
#else
  #define ROOM_STR  "Room 102"
  #define NODE_NAME "button_led_node_2"    
  #define TOPIC_PUB "/hospital/call/room2"
  #define TOPIC_SUB "/hospital/emergency_event/room2"
  // [v6.4] 발행 data: "102" (방 번호 숫자 통일)
  #define ROOM_DATA "102"
#endif

// 상태 정의
typedef enum {
  STATE_NORMAL,
  STATE_CALLING,
  STATE_EMERGENCY
} RoomState;

RoomState current_state = STATE_NORMAL;

// 에러 핸들러
#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while (1) {
    digitalWrite(PIN_LED_RED, !digitalRead(PIN_LED_RED));
    delay(100);
  }
}

// LCD 
void lcd_update(RoomState state) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(ROOM_STR);
  lcd.setCursor(0, 1);
  switch (state) {
    case STATE_NORMAL:    lcd.print("Normal");     break;
    case STATE_CALLING:   lcd.print("Calling..."); break;
    case STATE_EMERGENCY: lcd.print("EMERGENCY!"); break;
  }
}

// LED 
void led_update(RoomState state) {
  digitalWrite(PIN_LED_GREEN, state == STATE_NORMAL    ? HIGH : LOW);
  digitalWrite(PIN_LED_BLUE,  state == STATE_CALLING   ? HIGH : LOW);
  digitalWrite(PIN_LED_RED,   state == STATE_EMERGENCY ? HIGH : LOW);
}

// BUZZER
void buzzer_beep(int times) {
  for (int i = 0; i < times; i++) {
    digitalWrite(PIN_BUZZER, HIGH);
    delay(200);
    digitalWrite(PIN_BUZZER, LOW);
    delay(200);
  }
}

// BUTTON
void check_button() {
  static bool last_state = HIGH;
  static unsigned long last_press = 0;

  bool btn = digitalRead(PIN_BTN);

  if (btn == LOW && last_state == HIGH) {
    unsigned long now = millis();
    if (now - last_press > 300) {  // 디바운스 300ms
      last_press = now;

      // [v6.4] data: "101" or "102" (방 번호 숫자로 통일)
      static char data[8];
      snprintf(data, sizeof(data), "%s", ROOM_DATA);
      pub_msg.data.data     = data;
      pub_msg.data.size     = strlen(data);
      pub_msg.data.capacity = sizeof(data);
      RCSOFTCHECK(rcl_publish(&publisher_call, &pub_msg, NULL));

      // 상태 → CALLING
      current_state = STATE_CALLING;
      led_update(current_state);
      lcd_update(current_state);
      buzzer_beep(1);
    }
  }
  last_state = btn;
}

// emergency_event 구독 콜백
// [v6.4] 해당 방 토픽만 구독하므로 자동으로 해당 방만 제어됨
void event_callback(const void * msgin) {
  const std_msgs__msg__String * msg = (const std_msgs__msg__String *)msgin;
  String data = String(msg->data.data);

  if (data == "emergency") {
    current_state = STATE_EMERGENCY;
    led_update(current_state);
    lcd_update(current_state);
    buzzer_beep(3);

  } else if (data == "dispatching") {
    current_state = STATE_CALLING;
    led_update(current_state);
    lcd_update(current_state);

  } else if (data == "idle") {
    current_state = STATE_NORMAL;
    led_update(current_state);
    lcd_update(current_state);
  }
}

void setup() {
  set_microros_transports();

  pinMode(PIN_BTN,       INPUT_PULLUP);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE,  OUTPUT);
  pinMode(PIN_LED_RED,   OUTPUT);
  pinMode(PIN_BUZZER,    OUTPUT);

  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_BLUE,  LOW);
  digitalWrite(PIN_LED_RED,   LOW);
  digitalWrite(PIN_BUZZER,    LOW);

  lcd.init();
  lcd.backlight();
  lcd_update(STATE_NORMAL);
  led_update(STATE_NORMAL);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // 도메인 ID 5로 고정
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, 5));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // Publisher: /hospital/call/room1 or room2
  RCCHECK(rclc_publisher_init_default(
    &publisher_call, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    TOPIC_PUB));

  // Subscriber: /hospital/emergency_event/room1 or room2
  RCCHECK(rclc_subscription_init_default(
    &subscriber_event, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    TOPIC_SUB));

  static char sub_buf[64];
  sub_msg.data.data     = sub_buf;
  sub_msg.data.size     = 0;
  sub_msg.data.capacity = sizeof(sub_buf);

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber_event, &sub_msg, &event_callback, ON_NEW_DATA));
}

// loop
void loop() {
  check_button();
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)));
}

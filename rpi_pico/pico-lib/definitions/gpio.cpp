#include "../gpio.h"

void pinMode(const uint8_t pin, const uint8_t mode) {
    switch (mode) {
        case INPUT:
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_IN);
            break;
        case OUTPUT:
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_OUT);
            break;
        case INPUT_PULLUP:
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_IN);
            gpio_pull_up(pin);
            break;
        case INPUT_PULLDOWN:
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_IN);
            gpio_pull_down(pin);
            break;
        case OUTPUT_PWM:
            gpio_init(pin);
            gpio_set_function(pin, GPIO_FUNC_PWM);
            break;
        case INPUT_ADC:
            adc_init();
            adc_gpio_init(pin);
            break;
        default:
            break;
    }
}

void digitalWrite(const uint8_t pin, const uint8_t value) {
    gpio_put(pin, value);
}

bool digitalRead(const uint8_t pin) {
    return gpio_get(pin);
}

void analogWrite(const uint8_t pin, const uint8_t value) {
    const uint slice_num = pwm_gpio_to_slice_num(pin);

    pwm_set_wrap(slice_num, 255);
    pwm_set_chan_level(slice_num, PWM_CHAN_A + (pin % 2), value);
    pwm_set_enabled(slice_num, true);
}

void toggleFor(const unsigned long ms, const uint8_t pin) {
    digitalWrite(pin, HIGH);
    delay(ms);
    digitalWrite(pin, LOW);
}

void interrupts(const std::vector<uint8_t>& pins, const gpio_irq_callback_t callback) {
    for (const uint8_t& pin : pins) {
        gpio_set_irq_enabled_with_callback(
            pin,
            GPIO_IRQ_EDGE_RISE,
            true,
            callback);
    }
}



uint16_t analogRead(const uint8_t pin) {
    adc_select_input(pin);
    return adc_read();
}

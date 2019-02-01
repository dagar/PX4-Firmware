
namespace board
{
namespace SPI
{
namespace bus
{
class enum name : uint8_t
{
	sensors = 1,
	memory = 2,
	baro = 4,
	external1 = 5,
	external2 = 6
}


struct SPIDev {
	enum name name;


}

template <uint8_t _PORT, uint8_t _PIN>
struct GPIO {
	static constexpr PORT{_PORT}; // add base? to become real address (uint32_t*)
	static constexpr PIN{_PIN};   // shift? to become real bit (uint16_t)
}

class Register
{
	static constexpr uint32_t* addr = 0x123;

	// # define putreg32(v,a)        (*(volatile uint32_t *)(a) = (v))
	void set(uint32_t v)
	{
		(*(volatile uint32_t *)(addr) = (v))
	}

	void get() {
		// # define getreg32(a)          (*(volatile uint32_t *)(a))
		return (*(volatile uint32_t *)(addr));
	}


}


template <typename GPIO>
class ChipSelect {

	static constexpr uint32_t GPIO_SETUP{GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET};
	static constexpr uint32_t GPIO_OFF{GPIO_INPUT|GPIO_PULLDOWN|GPIO_SPEED_2MHz};

	ChipSelect() {
		px4_arch_configgpio(GPIO_SETUP|GPIO::PORT|GPIO::PIN);
	}

	~ChipSelect() {
		px4_arch_unconfiggpio(GPIO_SETUP|GPIO::PORT|GPIO::PIN);
	}

	void select() {
		// px4_arch_gpiowrite(gpio_setup|PORT|PIN, 0);
		// TODO: COMPILE DOWN to 1 instruction!!
		PORT + STM32_GPIO_BSRR_OFFSET = GPIO_BSRR_RESET(PIN)

		// MAYBE: Register<PORT + STM32_GPIO_BSRR_OFFSET> = GPIO_BSRR_RESET(PIN);
	}

	void clear() {
		// px4_arch_gpiowrite(gpio_setup|PORT|PIN, 1);
		// TODO: COMPILE DOWN to 1 instruction!!
		PORT + STM32_GPIO_BSRR_OFFSET = GPIO_BSRR_SET(PIN);
	}

	void off() { px4_arch_configgpio(GPIO_OFF|PORT|PIN); }
}


static constexpr ChipSelect<STM32_GPIOI_BASE, GPIO_PIN5> icm20602;

// GPIOPortI = STM32_GPIOI_BASE + STM32_GPIO_BSRR_OFFSET;
// need to preserve NuttX PORT and PIN for config and unconfig

// a gpio needs to be a static constexpr thing
// static constexpr GPIO<PortA, Pin17> GPIO_SPI4_CS1_MS5611 




} // namespace bus
} // namespace SPI
} // namespace px4::board
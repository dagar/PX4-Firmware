
I2CSPIDriverBase *
AK09916::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator, int runtime_instance)
{
	AK09916 *interface = new AK09916(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.rotation);

	if (interface == nullptr) {
		PX4_ERR("failed creating interface for bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	if (interface->init() != OK) {
		delete interface;
		PX4_DEBUG("no device on bus %i (devid 0x%x)", iterator.bus(), iterator.devid());
		return nullptr;
	}

	return interface;
}

void
AK09916::print_usage()
{
	PRINT_MODULE_USAGE_NAME("ak09916", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("magnetometer");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 0, 0, 35, "Rotation", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

int
ak09916_main(int argc, char *argv[])
{
	int ch;
	using ThisDriver = AK09916;
	BusCLIArguments cli{true, false};
	cli.default_i2c_frequency = 400000;

	while ((ch = cli.getopt(argc, argv, "R:")) != EOF) {
		switch (ch) {
		case 'R':
			cli.rotation = (enum Rotation)atoi(cli.optarg());
			break;
		}
	}

	const char *verb = cli.optarg();

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_MAG_DEVTYPE_AK09916);

	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	}

	if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	}

	if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return 1;
}

px4_add_module(
	MODULE modules__palletrone_mob
	MAIN palletrone_mob
	SRCS
		PalletroneMob.cpp
		PalletroneMob.hpp
		MomentumObserver.hpp
		palletrone_mob_physical_parms.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
)

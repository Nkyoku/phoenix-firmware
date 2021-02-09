#pragma once

#include <stdint.h>
#include <system.h>

class VectorController;

class VectorControllerStatus {
    friend class VectorController;
    
public:
    uint16_t Status;
    
    int AnyFault(void) {
        return ~Status & 0xFFF0;
    }
    
    int OverTemperatureFault(void) {
        // モーター1の障害フラグは0bit目、モーター2の障害フラグは1bit目といった順に格納して返す
        // 他の種類の障害のビット列も同様
        return (~Status >> 12) & 0xF;
    }
    
    int OverTemperatureFaultN(void) {
        return (Status >> 12) & 0xF;
    }

    int OverCurrentFault(void) {
        return (~Status >> 8) & 0xF;
    }
    
    int OverCurrentFaultN(void) {
        return (Status >> 8) & 0xF;
    }

    int HallSensorFault(void) {
        return (~Status >> 4) & 0xF;
    }
    
    int HallSensorFaultN(void) {
        return (Status >> 4) & 0xF;
    }

    int EncoderFault(void) {
        return ~Status & 0xF;
    }
    
    int EncoderFaultN(void) {
        return Status & 0xF;
    }

private:
    VectorControllerStatus(int status){
        Status = status;
    }
};

class VectorController {
private:
	static constexpr uint32_t BASE = VECTOR_CONTROLLER_MASTER_0_BASE;

	struct Register_t {
		volatile uint16_t STATUS;
		volatile uint16_t INTFLAG;
		volatile uint16_t FAULT;
		volatile uint16_t POSITION;
		volatile int16_t ENCODER1;
		volatile int16_t ENCODER2;
		volatile int16_t ENCODER3;
		volatile int16_t ENCODER4;
		volatile int16_t IMEASD1;
		volatile int16_t IMEASQ1;
		volatile int16_t IMEASD2;
		volatile int16_t IMEASQ2;
		volatile int16_t IMEASD3;
		volatile int16_t IMEASQ3;
		volatile int16_t IMEASD4;
		volatile int16_t IMEASQ4;
		volatile int16_t IREFD1;
		volatile int16_t IREFQ1;
		volatile int16_t IREFD2;
		volatile int16_t IREFQ2;
		volatile int16_t IREFD3;
		volatile int16_t IREFQ3;
		volatile int16_t IREFD4;
		volatile int16_t IREFQ4;
		volatile uint16_t KP;
		volatile uint16_t KI;
	};

public:
    static constexpr int GainScale = 14;
    
	static VectorControllerStatus GetStatus(void) {
        return VectorControllerStatus(__builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->STATUS));
	}

	static VectorControllerStatus GetInterruptFlag(void) {
		return VectorControllerStatus(__builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->INTFLAG));
	}

	static void SetFault(void) {
		__builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x1);
	}

	static void ClearFault(void) {
		__builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x2);
	}

	static void ResetFault(void) {
        __builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->FAULT, 0x3);
    }

	static bool IsFault(void) {
	    return __builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->FAULT) != 0;
	}

	static int GetPositionStatus(void) {
		return __builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->POSITION);
	}

	static int GetEncoderValue(int number) {
		switch(number){
		case 1:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->ENCODER1);
		case 2:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->ENCODER2);
		case 3:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->ENCODER3);
		case 4:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->ENCODER4);
		default:
			return 0;
		}
	}

	static int GetCurrentMeasurementD(int number) {
		switch(number){
		case 1:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASD1);
		case 2:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASD2);
		case 3:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASD3);
		case 4:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASD4);
		default:
			return 0;
		}
	}

	static int GetCurrentMeasurementQ(int number) {
		switch(number){
		case 1:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASQ1);
		case 2:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASQ2);
		case 3:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASQ3);
		case 4:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IMEASQ4);
		default:
			return 0;
		}
	}

	static int GetCurrentReferenceD(int number) {
		switch(number){
		case 1:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFD1);
		case 2:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFD2);
		case 3:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFD3);
		case 4:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFD4);
		default:
			return 0;
		}
	}

	static int GetCurrentReferenceQ(int number) {
		switch(number){
		case 1:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFQ1);
		case 2:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFQ2);
		case 3:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFQ3);
		case 4:
			return __builtin_ldhio(&reinterpret_cast<Register_t*>(BASE)->IREFQ4);
		default:
			return 0;
		}
	}

	static void SetCurrentReferenceD(int number, int value) {
		switch(number){
		case 1:
			__builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFD1, value);
			break;
		case 2:
			__builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFD2, value);
			break;
		case 3:
			__builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFD3, value);
			break;
		case 4:
			__builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFD4, value);
			break;
		default:
			break;
		}
	}

	static void SetCurrentReferenceQ(int number, int value) {
		switch(number){
		case 1:
			__builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFQ1, value);
			break;
		case 2:
			__builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFQ2, value);
			break;
		case 3:
			__builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFQ3, value);
			break;
		case 4:
			__builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->IREFQ4, value);
			break;
		default:
			break;
		}
	}

	static int GetGainP(void) {
		return __builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->KP);
	}

	static void SetGainP(int value) {
		__builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->KP, value);
	}

	static int GetGainI(void) {
		return __builtin_ldhuio(&reinterpret_cast<Register_t*>(BASE)->KI);
	}

	static void SetGainI(int value) {
		__builtin_sthio(&reinterpret_cast<Register_t*>(BASE)->KI, value);
	}
};

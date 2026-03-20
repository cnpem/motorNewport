/* drvXPSAsynAux.c */

/* This driver implements "auxilliary" commands for the XPS controller, i.e.
 * commands beyond those for the motor record.  These include support for
 * analog and digital I/O. */

#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include <cantProceed.h> /* !! for callocMustSucceed() */

#include <epicsExit.h>
#include <epicsMutex.h>
#include <epicsEvent.h>
#include <epicsThread.h>
#include <epicsString.h>
#include <errlog.h>
#include <iocsh.h>

#include <asynPortDriver.h>
#include <asynFloat64.h>
#include <asynUInt32Digital.h>

#include <epicsExport.h>
#include <XPS_C8_drivers.h>

#define analogInputString   "ANALOG_INPUT"
#define analogOutputString  "ANALOG_OUTPUT"
#define analogGainString    "ANALOG_GAIN"
#define binaryInputString   "BINARY_INPUT"
#define binaryOutputString  "BINARY_OUTPUT"

#define TCP_TIMEOUT 1.0

#define MAX_ANALOG_INPUTS   4
#define MAX_ANALOG_OUTPUTS  4
#define MAX_DIGITAL_INPUTS  4
#define MAX_DIGITAL_OUTPUTS 3
#define MAX_ADDR 3

static const char *driverName = "drvXPSAsynAux";

class epicsShareClass drvXPSAsynAux : public asynPortDriver
{
  public:
  /* These are the methods we override from the base class */
  drvXPSAsynAux(const char *portName, const char *ip, int port, int pollPeriod);
  asynStatus readUInt32D(asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask);
  asynStatus writeUInt32D(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask);
  asynStatus readFloat64(asynUser *pasynUser, epicsFloat64 *value);
  asynStatus writeFloat64(asynUser *pasynUser, epicsFloat64 value);
  void poller();
  bool shuttingDown_;

  private:
    int analogInput_;
    int analogOutput_;
    int analogGain_;
    int binaryInput_;
    int binaryOutput_;
    char *portName_;
    int socketID_;
    epicsEventId pollerEventId_;
    double pollerTimeout_;
};


static char *analogInputNames[MAX_ANALOG_INPUTS] = {
    "GPIO2.ADC1", /* Analog Input # 1 of the I/O board connector # 2 */
    "GPIO2.ADC2", /* Analog Input # 2 of the I/O board connector # 2 */
    "GPIO2.ADC3", /* Analog Input # 3 of the I/O board connector # 2 */
    "GPIO2.ADC4", /* Analog Input # 4 of the I/O board connector # 2 */
};
static char *analogOutputNames[MAX_ANALOG_OUTPUTS] = {
    "GPIO2.DAC1", /* Analog Output # 1 of the I/O board connector # 2 */
    "GPIO2.DAC2", /* Analog Output # 2 of the I/O board connector # 2 */
    "GPIO2.DAC3", /* Analog Output # 3 of the I/O board connector # 2 */
    "GPIO2.DAC4", /* Analog Output # 4 of the I/O board connector # 2 */
};
static char *digitalInputNames[MAX_DIGITAL_INPUTS] = {
    "GPIO1.DI", /* Digital Input of the I/O board connector # 1 (8 bits) */
    "GPIO2.DI", /* Digital Input of the I/O board connector # 2 (6 bits) */
    "GPIO3.DI", /* Digital Input of the I/O board connector # 3 (6 bits) */
    "GPIO4.DI", /* Digital Input of the I/O board connector # 4 (16 bits) */
};
static char *digitalOutputNames[MAX_DIGITAL_OUTPUTS] = {
    "GPIO1.DO", /* Digital Output of the I/O board connector # 1 (8 bits) */
    "GPIO3.DO", /* Digital Output of the I/O board connector # 3 (6 bits) */
    "GPIO4.DO", /* Digital Output of the I/O board connector # 4 (16 bits) */
};

static void shutdownCallbackC(void *pPvt);
static void XPSAuxPollerC(void *pPvt);

drvXPSAsynAux::drvXPSAsynAux(const char *portName, /* asyn port name */
                             const char *ip,       /* XPS IP address or IP name */
                             int port,             /* IP port number that XPS is listening on */
                             int pollPeriod)       /* Time to poll (msec) analog and digital inputs */
    : asynPortDriver(portName, MAX_ADDR,
        asynUInt32DigitalMask | asynFloat64Mask | asynDrvUserMask,
        asynUInt32DigitalMask | asynFloat64Mask,
        ASYN_CANBLOCK | ASYN_MULTIDEVICE, 1, 0, 0),

      portName_(epicsStrDup(portName)),
      pollerEventId_(epicsEventCreate(epicsEventEmpty)),
      pollerTimeout_(pollPeriod/1000.)

{
    epicsThreadId threadId;
    static const char *functionName = "drvXPSAsynAux";


    socketID_ = TCP_ConnectToServer((char *)ip, port, TCP_TIMEOUT);
    if (socketID_ < 0) {
        printf("%s::%s: error calling TCP_ConnectToServer\n", driverName, functionName);
        return;
    }

    /* Register a shutdown callback */
    epicsAtExit(shutdownCallbackC, this);
    
    createParam(analogInputString,  asynParamFloat64,       &analogInput_);
    createParam(analogOutputString, asynParamFloat64,       &analogOutput_);
    createParam(analogGainString,   asynParamUInt32Digital, &analogGain_);
    createParam(binaryInputString,  asynParamUInt32Digital, &binaryInput_);
    createParam(binaryOutputString, asynParamUInt32Digital, &binaryOutput_);

    threadId = epicsThreadCreate("XPSAuxPoller", epicsThreadPriorityMedium,
                                 epicsThreadGetStackSize(epicsThreadStackMedium),
                                 (EPICSTHREADFUNC)XPSAuxPollerC,
                                 this);
    if (threadId == NULL) {
        errlogPrintf("%s::%s epicsThreadCreate failed\n", driverName, functionName);
        return;
    }

    return;
}

static void shutdownCallbackC(void *pPvt)
{
    drvXPSAsynAux *pAux = static_cast<drvXPSAsynAux *>(pPvt);
    pAux->lock();
    pAux->shuttingDown_ = true;
    pAux->unlock();
}

static void XPSAuxPollerC(void *pPvt)
{
    drvXPSAsynAux *pAux = static_cast<drvXPSAsynAux *>(pPvt);
    pAux->poller();
}

asynStatus drvXPSAsynAux::readFloat64(asynUser *pasynUser, epicsFloat64 *value)
{
    int channel;
    int command = pasynUser->reason;
    char *GPIOName;
    int status;
    static const char *functionName = "readFloat64";

    if (shuttingDown_) return asynError;
    
    pasynManager->getAddr(pasynUser, &channel);

    if (command == analogInput_) {
        if ((channel < 0) || (channel >= MAX_ANALOG_INPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s::%s channel out of range=%d",
                          driverName, functionName, channel);
            return asynError;
        }
        GPIOName = analogInputNames[channel];
    } else if (command == analogOutput_) {
        if ((channel < 0) || (channel >= MAX_ANALOG_OUTPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s::%s channel out of range=%d",
                          driverName, functionName, channel);
            return asynError;
        }
        GPIOName = analogOutputNames[channel];
    } else {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "%s::%s invalid command=%d",
                      driverName, functionName, command);
        return asynError;
    }
    status = GPIOAnalogGet(socketID_, 1, GPIOName, value);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "%s::%s error calling GPIOAnalogGet=%d",
                      driverName, functionName, status);
        return asynError ;
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s::%s, value=%f\n", driverName, functionName, *value);
    return asynSuccess;
}


asynStatus drvXPSAsynAux::writeFloat64(asynUser *pasynUser, epicsFloat64 value)
{
    int channel;
    int command = pasynUser->reason;
    char *GPIOName;
    int status;
    static const char *functionName = "writeFloat64";

    if (shuttingDown_) return asynError;
    
    pasynManager->getAddr(pasynUser, &channel);

    if (command == analogOutput_) {
        if ((channel < 0) || (channel >= MAX_ANALOG_OUTPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s::%s channel out of range=%d",
                          driverName, functionName, channel);
            return asynError;
        }
        GPIOName = analogOutputNames[channel];
     } else {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "%s::%s invalid command=%d",
                      driverName, functionName, command);
        return asynError;
    }
    status = GPIOAnalogSet(socketID_, 1, GPIOName, &value);
    if (status) {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "%s::%s error calling GPIOAnalogSet=%d",
                      driverName, functionName, status);
        return asynError;
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s::%s, value=%f\n", driverName, functionName, value);
    return asynSuccess;
}

asynStatus drvXPSAsynAux::readUInt32D(asynUser *pasynUser, epicsUInt32 *value, epicsUInt32 mask)
{
    int channel;
    int command = pasynUser->reason;
    int status;
    unsigned short rawValue;
    static const char *functionName = "readUInt32D";

    if (shuttingDown_) return asynError;
    
    pasynManager->getAddr(pasynUser, &channel);

    if (command == binaryInput_) {
        if ((channel < 0) || (channel >= MAX_DIGITAL_INPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s::%s readBi channel out of range=%d",
                          driverName, functionName, channel);
            return asynError;
        }
        status = GPIODigitalGet(socketID_, digitalInputNames[channel], &rawValue);
        if (status) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s::%s error calling GPIODigitalGet=%d",
                          driverName, functionName, status);
            return asynError;
        }
        *value = rawValue & mask;
    } else if (command == binaryOutput_) {
        if ((channel < 0) || (channel >= MAX_DIGITAL_OUTPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s::%s readBo channel out of range=%d",
                          driverName, functionName, channel);
            return asynError;
        }
        status = GPIODigitalGet(socketID_, digitalOutputNames[channel], &rawValue);
        if (status) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s::%s error calling GPIODigitalGet=%d",
                          driverName, functionName, status);
            return asynError;
        }
        *value = rawValue & mask;
    } else if (command == analogGain_) {
        if ((channel < 0) || (channel >= MAX_ANALOG_OUTPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s::%s channel out of range=%d",
                          driverName, functionName, channel);
            return asynError;
        }
        status = GPIOAnalogGainGet(socketID_, 1, analogInputNames[channel], (int*)value);
        if (status) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s::%s error calling GPIOAnalogGainSet=%d",
                          driverName, functionName, status);
            return asynError;
        }
    } else {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "%s::%s invalid command=%d",
                      driverName, functionName, command);
        return asynError;
    }
    asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
              "%s::%s, value=%x\n", driverName, functionName, *value);
    return asynSuccess;
}

asynStatus drvXPSAsynAux::writeUInt32D(asynUser *pasynUser, epicsUInt32 value, epicsUInt32 mask)
{
    int channel;
    int command = pasynUser->reason;
    char *GPIOName;
    int status;
    static const char *functionName = "writeUInt32D";

    if (shuttingDown_) return asynError;
    
    pasynManager->getAddr(pasynUser, &channel);

    if (command == binaryOutput_) {
        if ((channel < 0) || (channel >= MAX_DIGITAL_OUTPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s::%s channel out of range=%d",
                          driverName, functionName, channel);
            return asynError;
        }
        GPIOName = digitalOutputNames[channel];
        status = GPIODigitalSet(socketID_, GPIOName, mask, value);
        if (status) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s::%s error calling GPIODigitalSet=%d",
                          driverName, functionName, status);
            return asynError;
        }
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s::%s, set binary output value=%d\n", driverName, functionName, value);
    } else if (command == analogGain_) {
        if ((channel < 0) || (channel >= MAX_ANALOG_OUTPUTS)) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s::%s channel out of range=%d",
                          driverName, functionName, channel);
            return asynError;
        }
        GPIOName = analogInputNames[channel];
        status = GPIOAnalogGainSet(socketID_, 1, GPIOName, (int*)&value);
        if (status) {
            epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                          "%s::%s error calling GPIOAnalogGainSet=%d",
                          driverName, functionName, status);
            return asynError;
        }
        asynPrint(pasynUser, ASYN_TRACEIO_DRIVER,
                  "%s::%s, set gain value=%d\n", driverName, functionName, value);
     } else {
        epicsSnprintf(pasynUser->errorMessage, pasynUser->errorMessageSize,
                      "%s::%s invalid command=%d",
                      driverName, functionName, command);
        return asynError;
    }
    return asynSuccess;
}


void drvXPSAsynAux::poller()
{
    char analogNames[100] = "";
    double analogValues[MAX_ANALOG_INPUTS];
    unsigned short digitalValue;
    int i;
    int status;
    static const char *functionName = "poller";

    /* Build strings with the names of the analog and digital inputs */
    for (i=0; i<MAX_ANALOG_INPUTS; i++) {
        strcat(analogNames, analogInputNames[i]);
        strcat(analogNames, ";");
    }

    while(1) {
        status = epicsEventWaitWithTimeout(pollerEventId_, pollerTimeout_);
        lock();
        if (shuttingDown_) break;
        status = GPIOAnalogGet(socketID_, MAX_ANALOG_INPUTS, analogNames, analogValues);
        if (status) {
            asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                      "%s::%s error calling GPIOAnalogGet=%d\n", driverName, functionName, status);
        }
        for (i=0; i<MAX_ANALOG_INPUTS; i++) {
            setDoubleParam(i, analogInput_, analogValues[i]);
            callParamCallbacks(i);
        }
        for (i=0; i<MAX_DIGITAL_INPUTS; i++) {
            status = GPIODigitalGet(socketID_, digitalInputNames[i], &digitalValue);
            if (status) {
                asynPrint(pasynUserSelf, ASYN_TRACE_ERROR,
                          "%s::%s error calling GPIODigitalGet=%d\n", driverName, functionName, status);
            }
            setUIntDigitalParam(i, binaryInput_, digitalValue, 0xffffffff);
            callParamCallbacks(i);
        }
        unlock();
    }
}

/** The following functions have C linkage, and can be called directly or from iocsh */

extern "C" {

static const iocshArg configArg0 = { "portName",iocshArgString};
static const iocshArg configArg1 = { "IP address",iocshArgString};
static const iocshArg configArg2 = { "IP port",iocshArgInt};
static const iocshArg configArg3 = { "polling period",iocshArgInt};
static const iocshArg * const configArgs[4] = {&configArg0,
                                               &configArg1,
                                               &configArg2,
                                               &configArg3};
static const iocshFuncDef configFuncDef = {"XPSAuxConfig",4,configArgs};
static void configCallFunc(const iocshArgBuf *args)
{
    new drvXPSAsynAux(args[0].sval, args[1].sval, args[2].ival, args[3].ival);
}

void drvXPSAsynAuxRegister(void)
{
    iocshRegister(&configFuncDef,configCallFunc);
}

epicsExportRegistrar(drvXPSAsynAuxRegister);

} // extern "C"

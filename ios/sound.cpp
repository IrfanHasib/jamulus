/******************************************************************************\
 * Copyright (c) 2004-2014
 *
 * Author(s):
 *  Volker Fischer
 *
 ******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE. See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
\******************************************************************************/

#include "sound.h"


/* Implementation *************************************************************/
CSound::CSound(
        void (*fpNewProcessCallback)(CVector<short> &psData, void *arg),
        void *arg,
        const int iCtrlMIDIChannel,
        const bool,
        const QString &) :
        CSoundBase("CoreAudio", true, fpNewProcessCallback, arg, iCtrlMIDIChannel) {


    // set up stream format
    streamFormat.mSampleRate = SYSTEM_SAMPLE_RATE_HZ;
    streamFormat.mFormatID = kAudioFormatLinearPCM;
    streamFormat.mFormatFlags = kAudioFormatFlagIsSignedInteger;
    streamFormat.mFramesPerPacket = 1;
    streamFormat.mBytesPerFrame = 4;
    streamFormat.mBytesPerPacket = 4;
    streamFormat.mChannelsPerFrame = 2; // stereo
    streamFormat.mBitsPerChannel = 16;

    // set up a callback struct for new input data
    inputCallbackStruct.inputProc = processInput;
    inputCallbackStruct.inputProcRefCon = this;

    // set up a callback struct for new output data
    outputCallbackStruct.inputProc = processOutput;
    outputCallbackStruct.inputProcRefCon = this;

    // allocate memory for buffer struct
    pBufferList = (AudioBufferList *) malloc(offsetof(AudioBufferList,
                                                      mBuffers[0]) + sizeof(AudioBuffer));

    // open the default unit
    /*ComponentDescription desc;*/
    AudioComponentDescription desc;
    desc.componentType = kAudioUnitType_Output;

    //Use "Remote IO" instead?? Core Audio Book says RemoteIO is HALOutput for iOS
    //seems depricated though... can't find usefull info about it in the docs
    //desc.componentSubType      = kAudioUnitSubType_HALOutput;
    desc.componentSubType = kAudioUnitSubType_RemoteIO;

    desc.componentManufacturer = kAudioUnitManufacturer_Apple;
    desc.componentFlags = 0;
    desc.componentFlagsMask = 0;

    AudioComponent comp = AudioComponentFindNext(NULL, &desc);
    if (comp == NULL) {
        qDebug() << "No CoreAudio next component found";
        throw CGenErr(tr("No CoreAudio next component found"));
    }

    if (AudioComponentInstanceNew(comp, &audioInputUnit)) {
        qDebug() << "CoreAudio creating input component instance failed";
        throw CGenErr(tr("CoreAudio creating input component instance failed"));
    }

    if (AudioComponentInstanceNew(comp, &audioOutputUnit)) {
        qDebug() << "CoreAudio creating output component instance failed";
        throw CGenErr(tr("CoreAudio creating output component instance failed"));
    }

    // we enable input and disable output for input component
    UInt32 enableIO = 1;
    AudioUnitSetProperty(audioInputUnit,
                         kAudioOutputUnitProperty_EnableIO,
                         kAudioUnitScope_Input,
                         1, // input element
                         &enableIO,
                         sizeof(enableIO));

    enableIO = 0;
    AudioUnitSetProperty(audioInputUnit,
                         kAudioOutputUnitProperty_EnableIO,
                         kAudioUnitScope_Output,
                         0, // output element
                         &enableIO,
                         sizeof(enableIO));

    // set up a callback function for new input data
    if (AudioUnitSetProperty(audioInputUnit,
                             kAudioOutputUnitProperty_SetInputCallback,
                             kAudioUnitScope_Global,
                             0,
                             &inputCallbackStruct,
                             sizeof(inputCallbackStruct))) {
        qDebug() << "CoreAudio audio unit set property failed";
        throw CGenErr(tr("CoreAudio audio unit set property failed"));
    }

    // set input stream format
    if (AudioUnitSetProperty(audioInputUnit,
                             kAudioUnitProperty_StreamFormat,
                             kAudioUnitScope_Output,
                             1,
                             &streamFormat,
                             sizeof(streamFormat))) {
        qDebug() << "CoreAudio stream format set property failed";
        throw CGenErr(tr("CoreAudio stream format set property failed"));
    }


    // set up a callback function for new output data
    if (AudioUnitSetProperty(audioOutputUnit,
                             kAudioUnitProperty_SetRenderCallback,
                             kAudioUnitScope_Global,
                             0,
                             &outputCallbackStruct,
                             sizeof(outputCallbackStruct))) {
        qDebug() << "CoreAudio audio unit set property failed";
        throw CGenErr(tr("CoreAudio audio unit set property failed"));
    }

    // ste output stream format
    if (AudioUnitSetProperty(audioOutputUnit,
                             kAudioUnitProperty_StreamFormat,
                             kAudioUnitScope_Input,
                             0,
                             &streamFormat,
                             sizeof(streamFormat))) {
        qDebug() << "CoreAudio stream format set property failed";
        throw CGenErr(tr("CoreAudio stream format set property failed"));
    }

    // always add system default devices for input and output as first entry
    lNumDevs = 0;
    strDriverNames[lNumDevs] = "System Default In/Out Devices";


    // TODO update to new APIs ala http://atastypixel.com/blog/using-remoteio-audio-unit/

    /**
    UInt32 iPropertySize = sizeof ( AudioDeviceID );
    if ( AudioHardwareGetProperty ( kAudioHardwarePropertyDefaultInputDevice,
                                    &iPropertySize,
                                    &audioInputDevice[lNumDevs] ) )
    {
        throw CGenErr ( tr ( "CoreAudio input AudioHardwareGetProperty call failed. "
                             "It seems that no sound card is available in the system." ) );
    }

    iPropertySize = sizeof ( AudioDeviceID );
    if ( AudioHardwareGetProperty ( kAudioHardwarePropertyDefaultOutputDevice,
                                    &iPropertySize,
                                    &audioOutputDevice[lNumDevs] ) )
    {
        throw CGenErr ( tr ( "CoreAudio output AudioHardwareGetProperty call failed. "
                             "It seems that no sound card is available in the system." ) );
    }

    lNumDevs++; // next device
    **/

    // init device index as not initialized (invalid)
    lNumDevs = 1;
    lCurDev = INVALID_SNC_CARD_DEVICE;
}

void CSound::GetAudioDeviceInfos(const AudioDeviceID DeviceID,
                                 QString &strDeviceName,
                                 bool &bIsInput,
                                 bool &bIsOutput) {
    /**a
    // get property name
    UInt32      iPropertySize = sizeof ( CFStringRef );

    CFStringRef sPropertyStringValue;

    AudioDeviceGetProperty ( DeviceID,
                             0,
                             false,
                             kAudioObjectPropertyName,
                             &iPropertySize,
                             &sPropertyStringValue );

    // convert CFString in c-string (quick hack!) and then in QString
    char* sC_strPropValue =
        (char*) malloc ( CFStringGetLength ( sPropertyStringValue ) + 1 );

    CFStringGetCString ( sPropertyStringValue,
                         sC_strPropValue,
                         CFStringGetLength ( sPropertyStringValue ) + 1,
                         kCFStringEncodingISOLatin1 );
    **/

    strDeviceName = strDriverNames[0];

    // check if device is input or output or both (is that possible?)
    // we do this by trying to set the current device for the audio unit
    // with the parameter input and output and then we simply check the
    // error/ok result
    bIsInput = !AudioUnitSetProperty(audioInputUnit,
                                     kAudioOutputUnitProperty_CurrentDevice,
                                     kAudioUnitScope_Global,
                                     1,
                                     &DeviceID,
                                     sizeof(AudioDeviceID));

    bIsOutput = !AudioUnitSetProperty(audioOutputUnit,
                                      kAudioOutputUnitProperty_CurrentDevice,
                                      kAudioUnitScope_Global,
                                      0,
                                      &DeviceID,
                                      sizeof(AudioDeviceID));
}

QString CSound::LoadAndInitializeDriver(int iDriverIdx) {
    qDebug() << "LoadAndInitializeDriver: " << iDriverIdx;

    // set input device
    /**
    if ( AudioUnitSetProperty ( audioInputUnit,
                                kAudioOutputUnitProperty_CurrentDevice,
                                kAudioUnitScope_Global,
                                1,
                                &audioInputDevice[iDriverIdx],
                                sizeof ( AudioDeviceID ) ) )
    {
        qDebug() << "CoreAudio input AudioUnitSetProperty call failed";
        throw CGenErr ( tr ( "CoreAudio input AudioUnitSetProperty call failed" ) );
    }

    // set output device
    if ( AudioUnitSetProperty ( audioOutputUnit,
                                kAudioOutputUnitProperty_CurrentDevice,
                                kAudioUnitScope_Global,
                                0,
                                &audioOutputDevice[iDriverIdx],
                                sizeof ( AudioDeviceID ) ) )
    {
        qDebug() << "CoreAudio output AudioUnitSetProperty call failed";
        throw CGenErr ( tr ( "CoreAudio output AudioUnitSetProperty call failed" ) );
    }
    **/

    // check device capabilities if it fullfills our requirements
    const QString strStat =
            CheckDeviceCapabilities(audioInputUnit, audioOutputUnit);

    qDebug() << "LoadAndInitializeDriver.strStat: " << strStat;

    // check if device is capable
    if (strStat.isEmpty()) {
        // store ID of selected driver if initialization was successful
        lCurDev = iDriverIdx;

        /**
// TODO why is only the input enough...?

        // setup callback for xruns (only for input is enough)
        AudioDeviceAddPropertyListener ( audioInputDevice[lCurDev],
                                         0,
                                         true,
                                         kAudioDeviceProcessorOverload,
                                         deviceNotification,
                                         this );
**/
    }


    return strStat;
}

QString CSound::CheckDeviceCapabilities(AudioComponentInstance &NewAudioInputUnit,
                                        AudioComponentInstance &NewAudioOutputUnit) {
    // BAD just assome there's no error and see what happens BAD
    return "";

    UInt32 size;

    // check input device sample rate
    size = sizeof(Float64);
    Float64 inputSampleRate = 0;
    AudioUnitGetProperty(NewAudioInputUnit,
                         kAudioUnitProperty_SampleRate,
                         kAudioUnitScope_Input,
                         1,
                         &inputSampleRate,
                         &size);

    if (static_cast<int> ( inputSampleRate ) != SYSTEM_SAMPLE_RATE_HZ) {
        return QString(tr("Current system audio input device sample "
                          "rate of %1 Hz is not supported. Please open the Audio-MIDI-Setup in "
                          "Applications->Utilities and try to set a sample rate of %2 Hz.")).arg(
                static_cast<int> ( inputSampleRate )).arg(SYSTEM_SAMPLE_RATE_HZ);
    }

    // check output device sample rate
    size = sizeof(Float64);
    Float64 outputSampleRate;
    AudioUnitGetProperty(NewAudioOutputUnit,
                         kAudioUnitProperty_SampleRate,
                         kAudioUnitScope_Output,
                         0,
                         &outputSampleRate,
                         &size);

    if (static_cast<int> ( outputSampleRate ) != SYSTEM_SAMPLE_RATE_HZ) {
        return QString(tr("Current system audio output device sample "
                          "rate of %1 Hz is not supported. Please open the Audio-MIDI-Setup in "
                          "Applications->Utilities and try to set a sample rate of %2 Hz.")).arg(
                static_cast<int> ( outputSampleRate )).arg(SYSTEM_SAMPLE_RATE_HZ);
    }

    // everything is ok, return empty string for "no error" case
    return "";
}

void CSound::CloseCoreAudio() {
    // clean up
    AudioUnitUninitialize(audioInputUnit);
    AudioUnitUninitialize(audioOutputUnit);
    AudioComponentInstanceDispose(audioInputUnit);
    AudioComponentInstanceDispose(audioOutputUnit);
}

void CSound::Start() {
    // start the rendering
    AudioOutputUnitStart(audioInputUnit);
    AudioOutputUnitStart(audioOutputUnit);

    // call base class
    CSoundBase::Start();
}

void CSound::Stop() {
    // stop the audio stream
    AudioOutputUnitStop(audioInputUnit);
    AudioOutputUnitStop(audioOutputUnit);

    // call base class
    CSoundBase::Stop();
}

int CSound::Init(const int iNewPrefMonoBufferSize) {
    UInt32 iActualMonoBufferSize;

    // Error message string: in case buffer sizes on input and output cannot be
    // set to the same value
    const QString strErrBufSize = tr("The buffer sizes of the current "
                                     "input and output audio device cannot be set to a common value. Please "
                                     "choose other input/output audio devices in your system settings.");

    // try to set input buffer size
    iActualMonoBufferSize =
            SetBufferSize(audioInputDevice[lCurDev], true, iNewPrefMonoBufferSize);

    if (iActualMonoBufferSize != static_cast<UInt32> ( iNewPrefMonoBufferSize )) {
        // try to set the input buffer size to the output so that we
        // have a matching pair
        if (SetBufferSize(audioOutputDevice[lCurDev], false, iActualMonoBufferSize) !=
            iActualMonoBufferSize) {
            qDebug() << "Init strErrBufSize " << strErrBufSize;
            throw CGenErr(strErrBufSize);
        }
    } else {
        // try to set output buffer size
        if (SetBufferSize(audioOutputDevice[lCurDev], false, iNewPrefMonoBufferSize) !=
            static_cast<UInt32> ( iNewPrefMonoBufferSize )) {
            qDebug() << "Init strErrBufSize " << strErrBufSize;
            throw CGenErr(strErrBufSize);
        }
    }

    // store buffer size
    iCoreAudioBufferSizeMono = iActualMonoBufferSize;

    // init base class
    CSoundBase::Init(iCoreAudioBufferSizeMono);

    // set internal buffer size value and calculate stereo buffer size
    iCoreAudioBufferSizeStereo = 2 * iCoreAudioBufferSizeMono;

    // create memory for intermediate audio buffer
    vecsTmpAudioSndCrdStereo.Init(iCoreAudioBufferSizeStereo);

    // fill audio unit buffer struct
    pBufferList->mNumberBuffers = 1;
    pBufferList->mBuffers[0].mNumberChannels = 2; // stereo
    pBufferList->mBuffers[0].mDataByteSize = iCoreAudioBufferSizeMono * 4; // 2 bytes, 2 channels
    pBufferList->mBuffers[0].mData = &vecsTmpAudioSndCrdStereo[0];

    // initialize units
    if (AudioUnitInitialize(audioInputUnit)) {
        qDebug() << "Initialization of CoreAudio failed";
        throw CGenErr(tr("Initialization of CoreAudio failed"));
    }

    if (AudioUnitInitialize(audioOutputUnit)) {
        qDebug() << "Initialization of CoreAudio failed";
        throw CGenErr(tr("Initialization of CoreAudio failed"));
    }

    return iCoreAudioBufferSizeMono;
}

UInt32 CSound::SetBufferSize(AudioDeviceID &audioDeviceID,
                             const bool bIsInput,
                             UInt32 iPrefBufferSize) {
    /**
    // first set the value
    UInt32 iSizeBufValue = sizeof ( UInt32 );
    AudioDeviceSetProperty ( audioDeviceID,
                             NULL,
                             0,
                             bIsInput,
                             kAudioDevicePropertyBufferFrameSize,
                             iSizeBufValue,
                             &iPrefBufferSize );

    // read back which value is actually used
    UInt32 iActualMonoBufferSize;
    AudioDeviceGetProperty ( audioDeviceID,
                             0,
                             bIsInput,
                             kAudioDevicePropertyBufferFrameSize,
                             &iSizeBufValue,
                             &iActualMonoBufferSize );

    return iActualMonoBufferSize;
    **/
    //Hardcode change sample rate to 48000 Hz
    Float64 sampleRate = 48000.0;
    UInt32 propSize = sizeof(Float64);
    AudioSessionSetProperty(kAudioSessionProperty_PreferredHardwareSampleRate,
                            propSize,
                            &sampleRate);

    AudioSessionGetProperty(kAudioSessionProperty_CurrentHardwareSampleRate,
                            &propSize,
                            &sampleRate);

    Float32 bufferDuration;
    propSize = sizeof(Float32);
    AudioSessionGetProperty(kAudioSessionProperty_CurrentHardwareIOBufferDuration,
                            &propSize,
                            &bufferDuration);

    UInt32 bufferLengthInFrames = sampleRate * bufferDuration;

    qDebug() << "Preferred buffer size:" << iPrefBufferSize;
    qDebug() << "Sample rate: " << sampleRate;
    qDebug() << "Buffer duration: " << bufferDuration;
    qDebug() << "Buffer length in Frames:" << bufferLengthInFrames;

    return bufferLengthInFrames;
}

OSStatus CSound::deviceNotification(AudioDeviceID,
                                    UInt32,
                                    Boolean,
                                    AudioUnitPropertyID inPropertyID,
                                    void *inRefCon) {
    /**
    CSound* pSound = static_cast<CSound*> ( inRefCon );


    if ( inPropertyID == kAudioDeviceProcessorOverload )
    {
        // xrun handling (it is important to act on xruns under CoreAudio
        // since it seems that the xrun situation stays stable for a
        // while and would give you a long time bad audio)
        pSound->EmitReinitRequestSignal ( RS_ONLY_RESTART );
    }
    **/

    return noErr;
}

OSStatus CSound::processInput(void *inRefCon,
                              AudioUnitRenderActionFlags *ioActionFlags,
                              const AudioTimeStamp *inTimeStamp,
                              UInt32 inBusNumber,
                              UInt32 inNumberFrames,
                              AudioBufferList *) {
    CSound *pSound = static_cast<CSound *> ( inRefCon );

    QMutexLocker locker(&pSound->Mutex);

    // get the new audio data
    AudioUnitRender(pSound->audioInputUnit,
                    ioActionFlags,
                    inTimeStamp,
                    inBusNumber,
                    inNumberFrames,
                    pSound->pBufferList);

    // call processing callback function
    pSound->ProcessCallback(pSound->vecsTmpAudioSndCrdStereo);

    return noErr;
}

OSStatus CSound::processOutput(void *inRefCon,
                               AudioUnitRenderActionFlags *,
                               const AudioTimeStamp *,
                               UInt32,
                               UInt32,
                               AudioBufferList *ioData) {
    CSound *pSound = static_cast<CSound *> ( inRefCon );

    QMutexLocker locker(&pSound->Mutex);

    memcpy(ioData->mBuffers[0].mData,
           &pSound->vecsTmpAudioSndCrdStereo[0],
           pSound->pBufferList->mBuffers[0].mDataByteSize);

    return noErr;
}


/**
Copyright (c) 
Audi Autonomous Driving Cup. All rights reserved.
 
Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:
 
1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.
3.  All advertising materials mentioning features or use of this software must display the following acknowledgement: �This product includes software developed by the Audi AG and its contributors for Audi Autonomous Driving Cup.�
4.  Neither the name of Audi nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY AUDI AG AND CONTRIBUTORS �AS IS� AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL AUDI AG OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


**********************************************************************
* $Author:: spiesra $  $Date:: 2015-05-13 08:29:07#$ $Rev:: 35003   $
**********************************************************************/
#include "stdafx.h"
#include "daedalusDriverModule.h"


ADTF_FILTER_PLUGIN("AADC State Controller", OID_ADTF_DAEDALUS_STATE_CONTROLLER, cDaedalusDriveModule);

#define SC_PROP_FILENAME "Maneuver File"


cDaedalusDriveModule::cDaedalusDriveModule(const tChar* __info) : cFilter(__info), m_bDebugModeEnabled(tFalse), m_hTimer(NULL)
{
    SetPropertyBool("Debug Output to Console",false);    
    SetPropertyStr("Debug Output to Console" NSSUBPROP_DESCRIPTION, "If enabled additional debug information is printed to the console (Warning: decreases performance)"); 

}

cDaedalusDriveModule::~cDaedalusDriveModule()
{
}

tResult cDaedalusDriveModule::Init(tInitStage eStage, __exception)
{
    RETURN_IF_FAILED(cFilter::Init(eStage, __exception_ptr));

    // pins need to be created at StageFirst
    if (eStage == StageFirst)    
    {
        
        cObjectPtr<IMediaDescriptionManager> pDescManager;
        RETURN_IF_FAILED(_runtime->GetObject(OID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             IID_ADTF_MEDIA_DESCRIPTION_MANAGER,
                                             (tVoid**)&pDescManager,
                                             __exception_ptr));
        /*
        * the MediaDescription for <struct name="tJuryNotAusFlag" .../> has to exist in a description file (e.g. in $ADTF_DIR\description\ or $ADTF_DIR\src\examples\src\description
        * before (!) you start adtf_devenv !! if not: the Filter-Plugin will not loaded because cPin.Create() and so ::Init() failes !
        */

        // input jury struct
        tChar const * strDesc1 = pDescManager->GetMediaDescription("tJuryStruct");
        RETURN_IF_POINTER_NULL(strDesc1);
        cObjectPtr<IMediaType> pType1 = new cMediaType(0, 0, 0, "tJuryStruct", strDesc1, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_JuryStructInputPin.Create("Jury_Struct", pType1, this));
        RETURN_IF_FAILED(RegisterPin(&m_JuryStructInputPin));
        RETURN_IF_FAILED(pType1->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionJuryStruct));
        
        // output driver struct
        tChar const * strDesc2 = pDescManager->GetMediaDescription("tDriverStruct");
        RETURN_IF_POINTER_NULL(strDesc2);
        cObjectPtr<IMediaType> pType2 = new cMediaType(0, 0, 0, "tDriverStruct", strDesc2, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_DriverStructOutputPin.Create("Driver_Struct", pType2, this));
        RETURN_IF_FAILED(RegisterPin(&m_DriverStructOutputPin));
        RETURN_IF_FAILED(pType2->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDriverStruct));

        // inputs for the decision module
        tChar const * strDescIdValue = pDescManager->GetMediaDescription("tIdValue");    
        RETURN_IF_POINTER_NULL(strDescIdValue);        

        cObjectPtr<IMediaType> pTypeIdValue = new cMediaType(0, 0, 0, "tIdValue", strDescIdValue,IMediaDescription::MDF_DDL_DEFAULT_VERSION);    
        RETURN_IF_FAILED(pTypeIdValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDecisionInput)); 
        RETURN_IF_FAILED(pTypeIdValue->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescriptionDecisionOutput));    
        
        RETURN_IF_FAILED(m_DecisionOutputPin.Create("Action",pTypeIdValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_DecisionOutputPin));
        
        RETURN_IF_FAILED(m_DecisionInputPin.Create("Feedback",pTypeIdValue, static_cast<IPinEventSink*> (this)));
        RETURN_IF_FAILED(RegisterPin(&m_DecisionInputPin));
        

        // input maneuver list
        tChar const * strDesc3 = pDescManager->GetMediaDescription("tManeuverList");
        RETURN_IF_POINTER_NULL(strDesc3);
        cObjectPtr<IMediaType> pType3 = new cMediaType(0, 0, 0, "tManeuverList", strDesc3, IMediaDescription::MDF_DDL_DEFAULT_VERSION);
        RETURN_IF_FAILED(m_ManeuverListInputPin.Create("Maneuver_List", pType3, this));
        RETURN_IF_FAILED(RegisterPin(&m_ManeuverListInputPin));
        RETURN_IF_FAILED(pType3->GetInterface(IID_ADTF_MEDIA_TYPE_DESCRIPTION, (tVoid**)&m_pDescManeuverList));
        
        m_bDebugModeEnabled = GetPropertyBool("Debug Output to Console");

        m_i16CurrentManeuverID = 0;

        m_state = stateCar_STARTUP; 

        m_i16SectionListIndex = 0;
        m_i16ManeuverListIndex = 0;

        m_maneuverListEnd = false;
    }
    else if(eStage == StageGraphReady)
    {
                
        // no ids were set yet
        m_bIDsDriverStructSet = tFalse;
        m_bIDsJuryStructSet = tFalse;
    }
    RETURN_NOERROR;
}

tResult cDaedalusDriveModule::Start(__exception)
{
    return cFilter::Start(__exception_ptr);
}

tResult cDaedalusDriveModule::Run(tInt nActivationCode, const tVoid* pvUserData, tInt szUserDataSize, ucom::IException** __exception_ptr/* =NULL */)
{
    if (nActivationCode == IRunnable::RUN_TIMER)
    {
        SendState(m_state,m_i16CurrentManeuverID);
    }   
    
    return cFilter::Run(nActivationCode, pvUserData, szUserDataSize, __exception_ptr);
}

tResult cDaedalusDriveModule::Stop(__exception)
{
    __synchronized_obj(m_oCriticalSectionTimerSetup);
    
    destroyTimer(__exception_ptr);

    return cFilter::Stop(__exception_ptr);
}

tResult cDaedalusDriveModule::Shutdown(tInitStage eStage, __exception)
{     
    return cFilter::Shutdown(eStage, __exception_ptr);
}

tResult cDaedalusDriveModule::OnPinEvent(IPin* pSource, tInt nEventCode, tInt nParam1, tInt nParam2, IMediaSample* pMediaSample)
{

    RETURN_IF_POINTER_NULL(pMediaSample);
    RETURN_IF_POINTER_NULL(pSource);

    if (nEventCode == IPinEventSink::PE_MediaSampleReceived  )
    {      
        //process the request to the jury struct input pin
        if (pSource == &m_JuryStructInputPin) 
        {
            tInt8 i8ActionID = -2;
            tInt16 i16entry = -1;
            
            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pDescriptionJuryStruct,pMediaSample,pCoder);
               
                // get the IDs for the items in the media sample 
                if(!m_bIDsJuryStructSet)
                {
                    pCoder->GetID("i8ActionID", m_szIDJuryStructI8ActionID);
                    pCoder->GetID("i16ManeuverEntry", m_szIDJuryStructI16ManeuverEntry);
                    m_bIDsJuryStructSet = tTrue;
                }      

                pCoder->Get(m_szIDJuryStructI8ActionID, (tVoid*)&i8ActionID);
                pCoder->Get(m_szIDJuryStructI16ManeuverEntry, (tVoid*)&i16entry);              
            }
            //change the state depending on the input
            // action_GETREADY --> stateCar_READY
            // action_START --> stateCar_RUNNING
            // action_STOP --> stateCar_STARTUP
            switch (juryActions(i8ActionID))
            {
                case action_GETREADY:
                    if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("State Controller: Received: Request Ready with maneuver ID %d",i16entry));
                    changeState(stateCar_READY);
                    setManeuverID(i16entry);
                    break;
                case action_START:
                    if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("State Controller: Received: Run with maneuver ID %d",i16entry));
                    if (i16entry == m_i16CurrentManeuverID)
                        changeState(stateCar_RUNNING);
                    else
                        {
                        LOG_WARNING("StateController: The id of the action_START corresponds not with the id of the last action_GETREADY");
                        //setManeuverID(i16entry);
                        changeState(stateCar_RUNNING);
                        }
                        
                        setManeuverID(i16entry);
                        sendAction();
                        if(m_bDebugModeEnabled) LOG_ERROR(m_sCurrentManeuver);
		    
                    break;
                case action_STOP:
                    if(m_bDebugModeEnabled)  LOG_INFO(cString::Format("State Controller: Received: Stop with maneuver ID %d",i16entry));
                    changeState(stateCar_STARTUP);
                    sendStopAction();
                    //resetSection();
                    break;
            }
        }

        else if (pSource == &m_ManeuverListInputPin && m_pDescManeuverList != NULL)
        {

            {   // focus for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pDescManeuverList,pMediaSample,pCoder);


                std::vector<tSize> vecDynamicIDs;

                // retrieve number of elements by providing NULL as first paramter
                tSize szBufferSize = 0;
                if(IS_OK(pCoder->GetDynamicBufferIDs(NULL, szBufferSize)))
                {
                    // create a buffer depending on the size element
                    tChar* pcBuffer = new tChar[szBufferSize];
                    vecDynamicIDs.resize(szBufferSize);
                    // get the dynamic ids (we already got the first "static" size element)
                    if (IS_OK(pCoder->GetDynamicBufferIDs(&(vecDynamicIDs.front()), szBufferSize)))
                    {
                        // iterate over all elements
                        for (tUInt32 nIdx = 0; nIdx < vecDynamicIDs.size(); ++nIdx)
                        {
                            // get the value and put it into the buffer
                            pCoder->Get(vecDynamicIDs[nIdx], (tVoid*)&pcBuffer[nIdx]);
                        }

                        // set the resulting char buffer to the string object
                        m_strManeuverFileString = (const tChar*) pcBuffer;
                    }

                    // cleanup the buffer
                    delete pcBuffer;
                }

            }

            // trigger loading maneuver list and update the ui
            loadManeuverList();
        }
        
        else if (pSource == &m_DecisionInputPin)
	{
	    tFloat32 timestamp;
            tInt32 action;
            tBool status;
	    
            {   // scope for sample read lock
                __adtf_sample_read_lock_mediadescription(m_pDescriptionDecisionInput, pMediaSample, pCoder);

                pCoder->Get("ui32Timestamp", (tVoid*)&timestamp);
                pCoder->Get("i32Value", (tVoid*)&action);
				    pCoder->Get("bIsExecuted", (tVoid*)&status);
            }
            
            if (status)					// Last action was performed successful
	    {

	      changeState(stateCar_READY); // Go to ready state
	      // TODO make changes here
	      incrementManeuverID();		// Get next maneuver id
	      
	      if (m_maneuverListEnd){
	      		changeState(stateCar_COMPLETE); // Go to complete state
	      }else{	      	
		      setManeuverID(m_i16CurrentManeuverID);
			  sendAction();
			  changeState(stateCar_RUNNING); // Go to ready state
	          if(m_bDebugModeEnabled) LOG_ERROR(m_sCurrentManeuver);
	      }
	    }
	}
    }
    RETURN_NOERROR;

}

tResult cDaedalusDriveModule::sendAction ()
{
   cObjectPtr<IMediaSample> pMediaSample;
   RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

   cObjectPtr<IMediaSerializer> pSerializer;
   m_pDescriptionDecisionOutput->GetMediaSampleSerializer(&pSerializer);
   tInt nSize = pSerializer->GetDeserializedSize();
   
   DAEDALUS::Maneuver eManeuver;
   if (m_sCurrentManeuver == "left")
     	   eManeuver = DAEDALUS::left;
   else if (m_sCurrentManeuver == "right")
   	  eManeuver = DAEDALUS::right;
   else if (m_sCurrentManeuver == "straight")
   	  eManeuver = DAEDALUS::straight;
   else if (m_sCurrentManeuver == "cross_parking")
   	  eManeuver = DAEDALUS::cross_parking;
   else if (m_sCurrentManeuver == "parallel_parking")
   	  eManeuver = DAEDALUS::parallel_parking;
   else if (m_sCurrentManeuver == "pull_out_left")
   	  eManeuver = DAEDALUS::pull_out_left;
   else if (m_sCurrentManeuver == "pull_out_right")
   	  eManeuver = DAEDALUS::pull_out_right;
   
   tBool bIsExecuted = false;
   tFloat32 timestamp = 0;

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {   // scope for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionDecisionOutput, pMediaSample, pCoder);

		  pCoder->Set("ui32Timestamp", (tVoid*)&timestamp);
        pCoder->Set("i32Value", (tVoid*)&eManeuver);
        pCoder->Set("bIsExecuted", (tVoid*)&bIsExecuted);
    }      
        
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_DecisionOutputPin.Transmit(pMediaSample);
   
   RETURN_NOERROR;
}

tResult cDaedalusDriveModule::sendStopAction ()
{
   cObjectPtr<IMediaSample> pMediaSample;
   RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

   cObjectPtr<IMediaSerializer> pSerializer;
   m_pDescriptionDecisionOutput->GetMediaSampleSerializer(&pSerializer);
   tInt nSize = pSerializer->GetDeserializedSize();
   
   DAEDALUS::Maneuver eManeuver;
   eManeuver = DAEDALUS::stop;
   
   tBool bIsExecuted = false;
   tFloat32 timestamp = 0;

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {   // scope for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionDecisionOutput, pMediaSample, pCoder);

		  pCoder->Set("ui32Timestamp", (tVoid*)&timestamp);
        pCoder->Set("i32Value", (tVoid*)&eManeuver);
        pCoder->Set("bIsExecuted", (tVoid*)&bIsExecuted);
    }      
        
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_DecisionOutputPin.Transmit(pMediaSample);
   
   RETURN_NOERROR;
}

tResult cDaedalusDriveModule::SendState(stateCar state, tInt16 i16ManeuverEntry)
{            
    __synchronized_obj(m_oCriticalSectionTransmit);
    
    cObjectPtr<IMediaSample> pMediaSample;
    RETURN_IF_FAILED(AllocMediaSample((tVoid**)&pMediaSample));

    cObjectPtr<IMediaSerializer> pSerializer;
    m_pDescriptionDriverStruct->GetMediaSampleSerializer(&pSerializer);
    tInt nSize = pSerializer->GetDeserializedSize();

    tInt8 bValue = tInt8(state);

    RETURN_IF_FAILED(pMediaSample->AllocBuffer(nSize));
    {   // focus for sample write lock
        __adtf_sample_write_lock_mediadescription(m_pDescriptionDriverStruct,pMediaSample,pCoder);

        // get the IDs for the items in the media sample 
        if(!m_bIDsDriverStructSet)
        {
            pCoder->GetID("i8StateID", m_szIDDriverStructI8StateID);
            pCoder->GetID("i16ManeuverEntry", m_szIDDriverStructI16ManeuverEntry);
            m_bIDsDriverStructSet = tTrue;
        }  

        pCoder->Set(m_szIDDriverStructI8StateID, (tVoid*)&bValue);
        pCoder->Set(m_szIDDriverStructI16ManeuverEntry, (tVoid*)&i16ManeuverEntry);
    }      
        
    pMediaSample->SetTime(_clock->GetStreamTime());
    m_DriverStructOutputPin.Transmit(pMediaSample);
        
    //debug output to console
    if(m_bDebugModeEnabled)  
    {
        switch (state)
            {
            case stateCar_ERROR:
                if(m_bDebugModeEnabled) LOG_INFO(cString::Format("State Controller Module: Send state: ERROR, Maneuver ID %d",i16ManeuverEntry));
                break;
            case stateCar_READY:
                if(m_bDebugModeEnabled) LOG_INFO(cString::Format("State Controller Module: Send state: READY, Maneuver ID %d",i16ManeuverEntry));
                break;
            case stateCar_RUNNING:
                if(m_bDebugModeEnabled) LOG_INFO(cString::Format("State Controller Module: Send state: RUNNING, Maneuver ID %d",i16ManeuverEntry));
                break;
            case stateCar_COMPLETE:
                if(m_bDebugModeEnabled) LOG_INFO(cString::Format("State Controller Module: Send state: COMPLETE, Maneuver ID %d",i16ManeuverEntry));                   
                break;
            case stateCar_STARTUP:
                break;
            }
    }
        
    RETURN_NOERROR;
}

tResult cDaedalusDriveModule::loadManeuverList()
{
    m_sectorList.clear();
    cDOM oDOM;
    oDOM.FromString(m_strManeuverFileString);
    cDOMElementRefList oSectorElems;
    cDOMElementRefList oManeuverElems;

    //read first Sector Elem
    if(IS_OK(oDOM.FindNodes("AADC-Maneuver-List/AADC-Sector", oSectorElems)))
    {                
        //iterate through sectors
        for (cDOMElementRefList::iterator itSectorElem = oSectorElems.begin(); itSectorElem != oSectorElems.end(); ++itSectorElem)
        {
            //if sector found
            tSector sector;
            sector.id = (*itSectorElem)->GetAttributeUInt32("id");
                
            if(IS_OK((*itSectorElem)->FindNodes("AADC-Maneuver", oManeuverElems)))
            {
                //iterate through maneuvers
                for(cDOMElementRefList::iterator itManeuverElem = oManeuverElems.begin(); itManeuverElem != oManeuverElems.end(); ++itManeuverElem)
                {
                    tAADC_Maneuver man;
                    man.id = (*itManeuverElem)->GetAttributeUInt32("id");
                    man.action = (*itManeuverElem)->GetAttribute("action");
                    sector.maneuverList.push_back(man);
                }
            }
    
            m_sectorList.push_back(sector);
        }
    }
    if (oSectorElems.size() > 0)
    {
        LOG_INFO("StateController: Loaded Maneuver file successfully.");
        m_i16SectionListIndex = 0;
        m_i16ManeuverListIndex = 0;
    }
    else
    {
        LOG_ERROR("StateController: no valid Maneuver Data found!");
        m_i16SectionListIndex = -1;
        m_i16ManeuverListIndex =-1;
        RETURN_ERROR(ERR_INVALID_FILE);
    }
   
   
    RETURN_NOERROR;
}

tResult cDaedalusDriveModule::changeState(stateCar newState)
{
    // if state is the same do nothing
    if (m_state == newState) RETURN_NOERROR;
   
    //to secure the state is sent at least one time
    SendState(newState, m_i16CurrentManeuverID);
   
    //handle the timer depending on the state
    switch (newState)
    {
    case stateCar_ERROR:
        destroyTimer();
        LOG_INFO("State ERROR reached");
        break;
    case stateCar_STARTUP:
        destroyTimer();
        LOG_INFO("State STARTUP reached");
        break;
    case stateCar_READY:
        createTimer();
        LOG_INFO(adtf_util::cString::Format("State READY reached (ID %d)",m_i16CurrentManeuverID));
        break;
    case stateCar_RUNNING:
        if (m_state!=stateCar_READY)
            LOG_WARNING("Invalid state change to Car_RUNNING. Car_READY was not reached before");
        LOG_INFO("State RUNNING reached"); 
        break;
    case stateCar_COMPLETE:
        destroyTimer();
        LOG_INFO("State COMPLETE reached");
        break;
    }
    
    m_state = newState;
    RETURN_NOERROR;
}

tResult cDaedalusDriveModule::createTimer()
{
     // creates timer with 0.5 sec
     __synchronized_obj(m_oCriticalSectionTimerSetup);
     // additional check necessary because input jury structs can be mixed up because every signal is sent three times
     if (m_hTimer == NULL)
     {
            m_hTimer = _kernel->TimerCreate(tTimeStamp(0.5*1000000), 0, static_cast<IRunnable*>(this),
                                        NULL, NULL, 0, 0, adtf_util::cString::Format("%s.timer", OIGetInstanceName()));
     }
     else
     {
        LOG_ERROR("Timer is already running. Unable to create a new one.");
     }
     RETURN_NOERROR;
}

tResult cDaedalusDriveModule::destroyTimer(__exception)
{
    __synchronized_obj(m_oCriticalSectionTimerSetup);
    //destroy timer
    if (m_hTimer != NULL) 
    {        
        tResult nResult = _kernel->TimerDestroy(m_hTimer);
        if (IS_FAILED(nResult))
        {
            LOG_ERROR("Unable to destroy the timer.");
            THROW_ERROR(nResult);
        }
        m_hTimer = NULL;
    }
    //check if handle for some unknown reason still exists
    else       
    {
        LOG_WARNING("Timer handle not set, but I should destroy the timer. Try to find a timer with my name.");
        tHandle hFoundHandle = _kernel->FindHandle(adtf_util::cString::Format("%s.timer", OIGetInstanceName()));
        if (hFoundHandle)
        {
            tResult nResult = _kernel->TimerDestroy(hFoundHandle);
            if (IS_FAILED(nResult))
            {
                LOG_ERROR("Unable to destroy the found timer.");
                THROW_ERROR(nResult);
            }
        }
    }

    RETURN_NOERROR;
}

tResult cDaedalusDriveModule::incrementManeuverID()
{
    //check if list was sucessfully loaded in init    
    if (m_i16ManeuverListIndex!=-1 && m_i16SectionListIndex!=-1)
    {
        //check if end of section is reached
        if (m_sectorList[m_i16SectionListIndex].maneuverList.size()>tUInt(m_i16ManeuverListIndex+1))
        {
            //increment only maneuver index
            m_i16ManeuverListIndex++;
            m_i16CurrentManeuverID++;
        }
        else
        {
            //end of section was reached and another section is in list
            if (m_sectorList.size() >tUInt(m_i16SectionListIndex+1))
            {
                //reset maneuver index to zero and increment section list index
                m_i16SectionListIndex++;
                m_i16ManeuverListIndex=0;
                m_i16CurrentManeuverID++;
                if (m_sectorList[m_i16SectionListIndex].maneuverList[m_i16ManeuverListIndex].id !=m_i16CurrentManeuverID)
                {
                    LOG_ERROR("State Controller: inconsistancy in maneuverfile detected. Please check the file ");
                }
            }
            else
            {
            	m_maneuverListEnd = true;
                LOG_ERROR("State Controller: end of maneuverlist reached, cannot increment any more");
            }
        }
    }
    else
    {
        LOG_ERROR("State Controller: could not set new maneuver id because no maneuver list was loaded");
    }
    
    if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Increment Manevuer ID: Sectionindex is %d, Maneuverindex is %d, ID is %d",m_i16SectionListIndex,m_i16ManeuverListIndex,m_i16CurrentManeuverID));

    RETURN_NOERROR;
}

tResult cDaedalusDriveModule::resetSection()
{
    //maneuver list index to zero, and current maneuver id to first element in list 
    m_i16ManeuverListIndex=0;
    m_i16CurrentManeuverID = m_sectorList[m_i16SectionListIndex].maneuverList[m_i16ManeuverListIndex].id;
   
    if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Reset section: Sectionindex is %d, Maneuverindex is %d, ID is %d",m_i16SectionListIndex,m_i16ManeuverListIndex,m_i16CurrentManeuverID));

    RETURN_NOERROR;
}

tResult cDaedalusDriveModule::setManeuverID(tInt maneuverId)
{   
    //look for the right section id and write it to section combobox
    for(unsigned int i = 0; i < m_sectorList.size(); i++)
    {
        for(unsigned int j = 0; j < m_sectorList[i].maneuverList.size(); j++)
        {
            if(m_i16CurrentManeuverID == m_sectorList[i].maneuverList[j].id)
            {            
                m_i16SectionListIndex = i;
                m_i16ManeuverListIndex = j;
                m_i16CurrentManeuverID = maneuverId;
                m_sCurrentManeuver = m_sectorList[i].maneuverList[j].action;
                if(m_bDebugModeEnabled) LOG_INFO(adtf_util::cString::Format("Sectionindex is %d, Maneuverindex is %d, ID is %d",m_i16SectionListIndex,m_i16ManeuverListIndex,m_i16CurrentManeuverID));
                break;
            }
        }
    }    
    RETURN_NOERROR;
}



tResult cDaedalusDriveModule::setManeuver()
{
    m_sCurrentManeuver = m_sectorList[m_i16SectionListIndex].maneuverList[m_i16ManeuverListIndex].action;
    RETURN_NOERROR;
}
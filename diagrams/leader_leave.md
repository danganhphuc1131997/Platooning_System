```mermaid
activityDiagram
    %% Activity: Execute Leader Departure Sequence
    
    partition "Block: Current Leader" {
        start
        :Accept Event Action\n<<User Leave Request>>;
        
        fork
            :Send Signal Action\n<<SERVER_LEAVE_NOTICE>>;
            note right: Notify Backend Server via\nNetwork Infrastructure
        fork again
            if (Decision: Platoon Empty?) then (Yes)
                :Action: Execute Solo Lane Change;
                :Activity Final;
                stop
            else (No)
                :Action: Identify Successor Node\n(Immediate Rear Vehicle);
                
                :Send Signal Action\n<<LEAVE_PLATOON>>;
                note right: Transmit to Successor ID
                
                :Action: Perform Physical Lane Change\n(Log "Change Lane...");
                
                :Action: Deactivate V2V Leader Service;
                
                :Activity Final;
                stop
            endif
        end fork
    }

    partition "Block: Backend Server" {
        :Accept Event Action\n<<SERVER_LEAVE_NOTICE>>;
        :Action: Log Service: Record Departure Event;
        :Action: Database: Update Platoon Manifest;
        :Flow Final;
        stop
    }

    partition "Block: Successor (Heir)" {
        :Accept Event Action\n<<LEAVE_PLATOON>>;
        note right: Triggered by Signal Reception
        
        if (Decision: Is Target Self?) then (Yes)
            fork
                :Action: Verify Safety Clearance;
                note right: Ensure Old Leader has departed
            fork again
                :Action: Terminate Follower Mode;
                note right: Unbind Client Socket
            end fork
            
            :Action: Instantiate Leader Mode;
            note right: Bind Server Port (UDP 8080)
            
            :Action: Switch Control Law\n(CACC -> Cruise Control);
            
            :Send Signal Action\n<<PLATOON_STATE>>;
            note right: Broadcast New Platoon Configuration
        else (No)
            :Flow Final;
            stop
        endif
    }

    partition "Block: Remote Followers" {
        :Accept Event Action\n<<PLATOON_STATE>>;
        
        if (Decision: Leader ID Changed?) then (Yes)
            :Action: Update Platoon Reference;
        endif
        
        :Action: Resume Following Logic;
        :Flow Final;
        stop
    }
```

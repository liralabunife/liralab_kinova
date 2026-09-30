    #include "ftSenseHandler.hpp"

    bool CanDevice::Open(const std::string& interfaceName)
    {
        fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);

        if (fd < 0) {PrintSuggestions(); return false;}


        struct ifreq ifr{};
        std::strcpy(ifr.ifr_name, interfaceName.c_str());

        if (ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {PrintSuggestions(); return false;}

        struct sockaddr_can addr{};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {PrintSuggestions(); return false;}


        this->StartTransmission();
        this->BlockTransmission();
        this->GetFullScale();
        this->SetDataRate(1000000);
        this->StartTransmission();

        return true;
    }

    bool CanDevice::Send(uint32_t id, uint8_t b0, uint8_t b1)
    {
        struct can_frame frame{};
        frame.can_id = id;
        frame.can_dlc = 2;
        frame.data[0] = b0;
        frame.data[1] = b1;

        return write(fd, &frame, sizeof(frame)) == sizeof(frame);
    }

    int CanDevice::Receive()
    {
        /* I Don't know wtf is this, just some ChatGPT stuff for rising a timeout error */
        bool result = read(fd, &in_frame, sizeof(in_frame)) > 0;
        
        /* From now on is the previous Receive */
        switch (in_frame.can_id)
        {
        case 0x3DA: //FORCE ID
        {
            // std::cout << std::hex << (unsigned short)in_frame.data[1] << std::endl;
            // std::cout << std::hex << (unsigned short)in_frame.data[0] << std::endl;
            unsigned short tmp;
            int currForce = 0;
            for (int i = 1; i < (unsigned short )in_frame.can_dlc; i = i + 2) {
                tmp = (((unsigned short)in_frame.data[i]) << 8) & 0xFF00;
                tmp += ((unsigned short )in_frame.data[i-1]) & 0x00FF;
                in_force_pre_scale[currForce] = tmp;
                currForce++;
            }

            force_readed = true;
            return 0;
        }
        case 0x3DB: // TORQUE ID
        {
            int currTorque = 0;
            int16_t tmp;
            for (int i = 1; i < (unsigned short )in_frame.can_dlc; i = i + 2)
            {
                tmp = (((unsigned short )in_frame.data[i]) << 8) & 0xFF00;
                tmp += ((unsigned short )in_frame.data[i-1]) & 0x00FF;

                in_torque_pre_scale[currTorque] = tmp;
                currTorque++;
            }
            torque_readed = true;
            return 0;
        }
        case 0x2D0: // RESPONSE_ID
            if(in_frame.can_dlc == 0)
                return 1;
            if (in_frame.can_dlc==4)
            {
                GetValues(&in_value);
                return 2;
            }
            break;
        }

        return -1;
    }


    int CanDevice::ReceiveAllForceAndTorque()
    {
        struct can_frame frame;
        bool received = false;

        while (true)
        {
            ssize_t n = recv(fd, &frame, sizeof(frame), MSG_DONTWAIT);

            if (n < 0)
            {
                // Coda vuota: è il comportamento atteso
                if (errno == EAGAIN || errno == EWOULDBLOCK)
                    break;

                // Errore reale
                return -1;
            }

            if (n != sizeof(struct can_frame))
                continue;

            received = true;

            switch (frame.can_id)
            {
            case 0x3DA: // FORCE
            {
                int currForce = 0;

                for (int i = 1; i < frame.can_dlc; i += 2)
                {
                    uint16_t tmp = ((uint16_t)frame.data[i] << 8) |
                                    (uint16_t)frame.data[i - 1];

                    in_force_pre_scale[currForce++] = tmp;
                }

                force_readed = true;
                break;
            }

            case 0x3DB: // TORQUE
            {
                int currTorque = 0;

                for (int i = 1; i < frame.can_dlc; i += 2)
                {
                    uint16_t tmp = ((uint16_t)frame.data[i] << 8) |
                                    (uint16_t)frame.data[i - 1];

                    in_torque_pre_scale[currTorque++] = tmp;
                }

                torque_readed = true;
                break;
            }

            default:
                // Ignora tutti gli altri messaggi
                break;
            }
        }

        return received ? 0 : 1;   // 0 = almeno un frame letto, 1 = nessun frame disponibile
    }

    void CanDevice::BlockTransmission()
    {
        Send(0x20D, 0x7, 0x1);

        int returnValue = Receive();
        //read pack untill the ack
        while ((returnValue != 1))
            returnValue = Receive();
    }

    void CanDevice::StartTransmission()
    {
        Send(0x20D, 0x7, 0x0);
        int returnValue = Receive();

        //read pack untill the ack
        while ((returnValue != 1))
        {
            returnValue = Receive();
        }
    }

    void CanDevice::SetDataRate(int dataRate)
    {
        __u8 data_rate = (__u8)dataRate;

        Send(0x20D, 0x8, data_rate);
        int returnValue = Receive();
        //read pack untill the ack
        while ((returnValue != 1))
            returnValue = Receive();    
    }

    void CanDevice::GetFullScale() {
              
        __u8 B0,B1;
        int return_value;
        for (int count = 0; count < 6; ++count){
            switch (count){
                case 0: B1 = 0x0; break;    // 1566
                case 1: B1 = 0x1; break;    // 1776
                case 2: B1 = 0x2; break;    // 2122
                case 3: B1 = 0x3; break;    // 37
                case 4: B1 = 0x4; break;    // 37
                case 5: B1 = 0x5; break;    // 23
            }

            B0 = 0x18;
            Send(0x20D, B0, B1); 
            return_value = Receive();
            if (return_value < 0) 
                std::cout << "Get Full Scale Error!\n";
            scales_vect[count] = in_value;
        }

        for(int i = 0; i <6; i++)
            std::cout << i << ":" << scales_vect[i] << std::endl;
               
    }

    CanDevice::~CanDevice()
    {
        if (fd >= 0)
            close(fd);
    }
    can_frame CanDevice::GetInFrame() {return in_frame;}

    bool CanDevice::IsWrenchReady() 
    {
        if(force_readed && torque_readed)
        {
            force_readed = false;
            torque_readed = false;
            return true;   
        }
        return false;
    }
    
    void CanDevice::GetWrench(double* wrench)
    {
        for(int i = 0; i < 3; i++)
        {
            float scale = 1.0;
            if(i == 0)
            {
                scale = 1566.0;
            }
            if(i == 1)
            {
                scale = 1776;
            }
            if(i == 2)
            {
                scale = 2122;
            }
            wrench[i] = in_force_pre_scale[i] * scale/pow(2,15) - scale - current_tare[i];
        }

        for(int i = 3; i < 6; i++)
        {
            wrench[i] = in_torque_pre_scale[i - 3] * scales_vect[i]/pow(2,15) - scales_vect[i] - current_tare[i];
        }

        if(set_tare)
        {
            for(int i = 0; i < 6; i++)
                current_tare[i] = wrench[i];
            set_tare = false;
        }
        
        wrench[0] *= -1.0;
        wrench[3] *= -1.0;

        return;
    }
    
    void CanDevice::GetWrenchCompensated(double* wrench, KinovaLiralab::RobotState robotState, bool subtract_gravity)
    {
        for(int i = 0; i < 3; i++)
        {
            float scale = 1.0;
            if(i == 0)
            {
                scale = 1566.0;
            }
            if(i == 1)
            {
                scale = 1776;
            }
            if(i == 2)
            {
                scale = 2122;
            }
            wrench[i] = in_force_pre_scale[i] * scale/pow(2,15) - scale;
        }

        for(int i = 3; i < 6; i++)
        {
            wrench[i] = in_torque_pre_scale[i - 3] * scales_vect[i]/pow(2,15) - scales_vect[i];
        }

        wrench[0] *= -1.0;
        wrench[3] *= -1.0;

        for(int i = 0; i < 6; i++)
            wrench[i] -= current_tare[i];
        
        if(!compensationIsReady) {perror("Compensation is not ready. Call InitCompensation() before."); return;}

        Eigen::Matrix3d R_world_sensor;


        R_world_sensor <<
            robotState._eePose[3], robotState._eePose[4], robotState._eePose[5],
            robotState._eePose[6], robotState._eePose[7], robotState._eePose[8],
            robotState._eePose[9], robotState._eePose[10], robotState._eePose[11];

        Eigen::Vector3d g_sensor = R_world_sensor.transpose() * g_world;

        Eigen::Vector3d Fg = payload_mass * g_sensor;
        Eigen::Vector3d Tg = com_sensor_payload.cross(Fg);

        if (true)
        {
            for(int i = 0; i < 3; i++)
            {
                wrench[i] = wrench[i] - Fg[i];
                wrench[i+3] = wrench[i+3] - Tg[i];
            }
        }
        else
        {
            for(int i = 0; i < 3; i++)
            {
                wrench[i] = wrench[i] + Fg[i];
                wrench[i+3] = wrench[i+3] + Tg[i];
            }
        }

        if(set_tare)
        {
            for(int i = 0; i < 6; i++)
                current_tare[i] = wrench[i];
            set_tare = false;

            for(int i = 0; i < 6; i++)
                wrench[i] -= current_tare[i];
        }

        /*
        const double rad2deg = 180.0 / M_PI;
        Eigen::Vector3d xyz = R_world_sensor.eulerAngles(0,1,2);
        std::cout << "XYZ:\n";
        std::cout << "Roll  = " << xyz(0)*rad2deg << '\n';
        std::cout << "Pitch = " << xyz(1)*rad2deg << '\n';
        std::cout << "Yaw   = " << xyz(2)*rad2deg << '\n';
        */
       //printf("FG: %f, %f, %f", Fg[0], Fg[1], Fg[2]);
        return;
    }

    void CanDevice::SetTare(){set_tare = true;}

    void CanDevice::InitCompensation()
    {
        Eigen::Vector3d com_sensor_payload{0.0, 0.0, 0.08};
        Eigen::Isometry3d T_ee_sensor = Eigen::Isometry3d::Identity();
        Eigen::Vector3d translation(0.0, 0.0, 0.145);
        T_ee_sensor = T_ee_sensor.translate(translation);
        float payloadMass = 0.320 + 0.092;

        this->com_sensor_payload = com_sensor_payload;
        this->payload_mass = payloadMass;
        this->compensationIsReady = true;


        while (!this->IsWrenchReady())
        {
            this->Receive();
            std::cout << "Waiting for can msgs...\n";
        }
    }

    void CanDevice::GetValues(unsigned short int *value){
        short int outgoing;
        outgoing = (((unsigned short )in_frame.data[2]) << 8) & 0xFF00;
        outgoing += ((unsigned short )in_frame.data[3]) & 0x00FF; 
        *value=outgoing;
    }

    void CanDevice::PrintSuggestions()
    {
        perror("Cannot open CAN. Try with:"
            "\n\tsudo rmmod pcan"
            "\n\tsudo modprobe peak_usb"
            "\n\tsudo ip link set can0 up type can bitrate 1000000\n");
    }

    void CanDevice::PrintCurrentTare()
    {
        printf("TARE: %f, %f, %f", current_tare[0], current_tare[1], current_tare[2]);
    }
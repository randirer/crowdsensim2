/*
 * Simulation.cc

 *
 *  Created on: 30 giu 2016
 *      Author: Giuseppe Cacciatore
 */

#include "../Headers/Simulation.h"

#include "../Headers/Utilities.h"
#include "../Headers/ReadValues.h"


vector<float> simulationOperations(int num_users,Smartphones smM, Smartphones::iterator it_smM,Events eventsL, Events::iterator it_eventsL, Users usersM, Users::iterator it_usersM,int days){


	// WiFi
	double rho_tx=0.27; // WiFi power in transmission (W)
	double rho_id=3.68; // WiFi power in idle mode (W)
	double lambda_g=1000.0; // Rate of generation of packets
	double gamma_g=0.11*pow(10,-3); // Energy cost to elaborate a generated packet (J)
	int wifi_uplink_data_tx=1000000;// WiFi uplink data rate: 1 Mbps

	float datatotalitotgiorni=0.0;
	vector<float> results;
	float datatotali=0.0;
	float cons_sensing=0.0;
	float cons_report=0.0;

	double acc_sample_frequency=10; // Hz
	double acc_sample_size=6*8;// bit
	int acc_current=450;// uA
	bool flagstart=true;
/*	double tem_sample_frequency=50; // Hz
	double tem_sample_size=16;// bit
	int tem_current=1*tem_sample_frequency;*/// uA

	pair<float,float> locstop;

	double prox_sample_frequency=10; // Hz
	double prox_sample_size=2*8;// bit
	int prox_current=150;// uA

	int packtotali=0;
	double gps_sample_frequency=0.1; // Hz
	double gps_sample_size=24*8;// bit
	int gps_current=22000;// uA
	float totcontribution=0;
	float quadcontribution=0;
	float batcontrib=0;
	float quadbatcontrib=0;
	float datasent=0;
	float dataexpected;



	
	// Real consumption per minute, 1- thx while sensing 2- transfer after sensing 3- random thx
	static const float arr[] = {2.22,0.917,1.18,0.561};
	// probabilistic soglia 0.25= 1.22333   0.5= 1.08
	 float pbrep_cost=1.8;
	 float threshold=0.5;
	 float threshold2=0.2;

	 //soglia probabilistic
	 float reporting_cost= (2.011*11) /1200;  // costo di report per minute (mah/minute)
	 int bytes_per_minute=40; //kB per minute
	 float bat_threshold=1; //soglia conitnuous
	 int kbpermin=109;//kb tx per min
	 float b=1;
	 float upbound=1;
	 float lowbound=0.20;


	vector<float> cons (arr, arr + sizeof(arr) / sizeof(arr[0]) );
	static const string dcf[] = {"DDF","PDA","PCS"};
    int ctx=0;

    Smartphones active;
    Smartphones stopped;




    float sb;
    float cap;
    DatasentM temp_dtsM;
    DatatbdelM temp_dtbdelM;
    Samples temp_s;
    float old_battery;
    float enTot;
    Smartphone sm(0,0,0,0,"",temp_dtsM,temp_dtbdelM,temp_s,0);
    vector<pair<float, float> > route;
    vector<bool> generate;
    bool gen;
	float ref_lat;
	float ref_lon;
	bool pda=false;

    /* - - - - - - - - - - - - - - - - - - - */
	 // SIMULATION
	 /* - - - - - - - - - - - - - - - - - - - */


	int typeCons=readTypeCons();
    //int typeCons = 0;


	cout <<" USING THE DATA COLLECTION FRAMEWORK: "<< dcf[typeCons] << endl;

	if(typeCons==2)
		typeCons=3;

	if(typeCons==1){
		typeCons=2;
		pda=true;
	}




	ofstream outputsimdata;
	 ofstream stopfile;
	 ofstream activefile;
	 ofstream routesfile;
	ofstream  fairfile;
	ofstream  batfairfile;
	ofstream tres;
	ifstream perc;
	 ifstream dur;
	ostringstream fileNameStream;                    // let this be empty
	fileNameStream << "./Outputs/DataFairness_" << typeCons << ".txt"; // and pass "dice_" here
	string fileName = fileNameStream.str(); 
	fairfile.open(fileName.c_str(),ios::app);
	
	ostringstream fileNameStream2;                    // let this be empty
	fileNameStream2 << "./Outputs/BatFairness_" << typeCons << ".txt"; // and pass "dice_" here
	string fileName2 = fileNameStream2.str(); 
	batfairfile.open(fileName2.c_str(),ios::app);
	 outputsimdata.open("./Outputs/SimulationData.txt");
	 stopfile.open("./Outputs/Stopped.txt");
	 activefile.open("./Outputs/Active.txt");
	 routesfile.open("./Outputs/Routes.txt");
	 tres.open("./Outputs/tres.txt");
	 perc.open("./Inputs/perc_connections");
	 dur.open("./Inputs/durations");

	 outputsimdata << "/Day/-" << "/Hour/-" << "/Minute/-" << "/ID-User/-" << "/Lat/-"
			 	   << "/Long/- "<< "/BatteryLevel/-" << "/SensorDataInformation/-" << endl;
	 stopfile << "/ID-User/-" <<  "/DataGenerated/- "<< "/EnergyConsumption/-" << "/BatteryConsumption/-"<< "/BatteryLevel /- last position" << endl;
	 activefile << "/ID-User/-" <<  "/DataGenerated/- "<< "/EnergyConsumption/-"<< "/BatteryConsumption/-"<< "/BatteryLevel/- Minutes of Contribution -/ Timestamp -/ last position" << endl;
	 int currentDay=0;
	 vector<float> percentages;
	 vector<int> duration_hour;
	 int duration =0;
	 bool traces;
	 float volt=3.7;
	 int packold=0;
	 int numgeneration;

	 int counteruser=0;
	 //for(int currentDay = 0; currentDay<days;currentDay++){
	 int ref_user=-1;

	 int temp_traces=readBatteryDecision();

	 if(temp_traces==1)
		 traces=false;
	 else
		 traces=true;


			cout << "**Start simulation**" << endl;
		 if(typeCons==3){

		float tmp;

		 while(perc >> tmp){
			percentages.push_back(tmp);
		    //cout<< tmp<<endl;
		 }

		 while(dur >> tmp){
			 	 	duration_hour.push_back(tmp);
		 		    //cout<< tmp<<endl;
		 		 }




		 }

		 if(traces==false)
		 {	 kbpermin=((60*wifi_uplink_data_tx)/8); //Bytes
		 	 dataexpected=106028*num_users;
		 }
		 else
			 dataexpected=555.0125*num_users;



		 for (it_eventsL= eventsL.begin(); it_eventsL != eventsL.end(); ++it_eventsL) {




			 if(ref_user!=(*it_eventsL).id_user){
				 if(ref_user==-1){
					 ref_user=1;
				 }

				 else{
					 smM.erase(ref_user);
					 enTot=0;

					 float dati;
					 sb=sm.startBattery;
					 if(typeCons==0 || typeCons==1)
						 dati=temp_dtbdelM["Total"];
					 if(typeCons==2 || typeCons==3)
						 dati=temp_dtsM["Total"];

					 if(traces==true){
						 if(typeCons==1 ){

							ctx=dati * reporting_cost;
							enTot+=ctx;
						}

						if(typeCons==3 || pda==true ){

							ctx=numgeneration * pbrep_cost;
							enTot+=ctx;
							enTot+= cons[3]*(temp_dtbdelM["Minutes"]-numgeneration);
							cons_sensing+=cons[3]*(temp_dtbdelM["Minutes"]-numgeneration);
							cons_report+=ctx;
						}
						else{
						 enTot+= cons[typeCons]*temp_dtbdelM["Minutes"];
						 cons_sensing+=cons[3]*temp_dtbdelM["Minutes"];
						 cons_report+=enTot-cons[3]*temp_dtbdelM["Minutes"];
						}
					 }
					else{

						dati= temp_dtbdelM["data"]/1000;    // from Byte to KB
						enTot=temp_dtbdelM["consumption-wifi"]+ temp_dtbdelM["consumption-data"];
						cons_sensing+=temp_dtbdelM["consumption-data"];
						cons_report+=temp_dtbdelM["consumption-wifi"];

						if(typeCons==2 || typeCons==3){
							dati=temp_dtsM["Total"]/1000;
							enTot=temp_dtbdelM["Energy"];
							cons_sensing+=temp_dtbdelM["consumption-data"];
							cons_report+=enTot-temp_dtbdelM["consumption-data"];
						}
					}
					old_battery=sm.battery;
					float rem= (cap/100)*old_battery;
				 	rem = rem - enTot;

					old_battery= (rem/cap)*100;

					Smartphone temp_str(ref_user,sb,old_battery,cap,"Outdoor",temp_dtbdelM,temp_dtsM,temp_s,0);
					smM.insert(pair <int, Smartphone >(ref_user,temp_str) );
					it_eventsL--;
					time_t timeSinceEpoch = mktime(&((*(it_eventsL)).timestamp));
					if(locstop.first==0){
						locstop=make_pair(ref_lon,ref_lat);
					}
					datatotali+=dati;
					datatotalitotgiorni+=dati;

					float mintotali=(*it_usersM).second.minutes;
					if(mintotali!=0){
					float contrib=numgeneration/mintotali;
					float batteryrel= enTot/(rem + enTot);
					batcontrib+=batteryrel;
					quadbatcontrib+=batteryrel*batteryrel;
					totcontribution +=contrib;
					quadcontribution+=contrib*contrib;
					}
					activefile << sm.id_user <<" "<< dati<<" "<<enTot <<" "<< sb-old_battery <<" "<<old_battery << " "<<numgeneration << " "<< timeSinceEpoch<< " "<<locstop.first<<" "<<locstop.second <<" "<< temp_dtbdelM["consumption-wifi"]<< " "<<(*it_usersM).second.minutes << endl;
					it_eventsL++;
					int k=0;
					//if(ref_user==23 || ref_user==85||ref_user==119 ||ref_user==53 ||ref_user==106 )
					float datigenpermin=dati/numgeneration;
					float dg=0.0;
					for(int i=0;i<(route.size()-1);i++){
						dg=0.0;
						if(generate[i+1]==true)
							dg=datigenpermin;
						routesfile <<ref_user<<"	"<<"LINESTRING ("<<route[i].first<<" "<<route[i].second<<", "<<route[i+1].first<<" "<<route[i+1].second<<")"  << "	"<< generate[i+1] <<"	"<<dg<<endl;
						//routesfile <<ref_user<<"	"<<"LINESTRING ("<<route[i].first<<" "<<route[i].second<<", "<<route[i+1].first<<" "<<route[i+1].second<<")"  << "	"<< generate[i] <<endl;

					}
					 //cout <<"Pacchiiii   "<< packtotali-packold<<endl;
					 packold=packtotali;
					 temp_s.clear();
					 temp_dtsM.clear();
					 temp_dtbdelM.clear();
					 locstop=make_pair(0,0);

				 }



				 ref_user=(*it_eventsL).id_user;
				 it_smM=smM.find(ref_user);
				 it_usersM=usersM.find(ref_user);
				 sm=(*it_smM).second;
				 temp_dtbdelM=sm.dtbd;
				 temp_s=sm.smp;
				 temp_dtsM=sm.dts;
				 old_battery=sm.battery;
				 cap=sm.capacity;
				 generate.clear();
				 gen=true;
				 duration=0;
				 numgeneration=0;
				 flagstart=true;
				 route.clear();

			 }
			 ref_lat=(*it_eventsL).loc.lat;
			 ref_lon=(*it_eventsL).loc.lon;
			 float ref_alt=(*it_eventsL).loc.alt;





			 int ref_smartphone=sm.id_user;
			 string ref_context=sm.context;



			 tm prev_timestamp;
			 Samples::iterator it_tmpsample;

			 it_tmpsample= --temp_s.end();
			 //inserted to avoid for


			 if(currentDay!=(*it_eventsL).timestamp.tm_wday){
				 
				 int userusati=num_users;
				 results.push_back(datatotali/userusati);
				 results.push_back(cons_sensing/userusati);
				 results.push_back(cons_report/userusati);
				 cons_report=0;
				 cons_sensing=0;
				 datatotali=0;

				 counteruser=ref_user;
			 }



			 currentDay=(*it_eventsL).timestamp.tm_wday;

			 prev_timestamp=(*it_tmpsample).pos.timestamp;
			 //serve???
			 //prev_timestamp.tm_wday = currentDay;

			 // if sample list empty previous timestamp = actual event ts
			 if(temp_s.size() == 0){
				 prev_timestamp=(*it_eventsL).timestamp;
			 }

			 // edited first event of the day has to be 0 as difference of minutes between samples
			 int time_between_samples = 0;
			 if((*it_eventsL).timestamp.tm_wday == prev_timestamp.tm_wday){
				 if((*it_eventsL).timestamp.tm_hour == prev_timestamp.tm_hour){
					 time_between_samples = abs((*it_eventsL).timestamp.tm_min-prev_timestamp.tm_min);
				 }


				 if((*it_eventsL).timestamp.tm_hour == prev_timestamp.tm_hour+1){
					 time_between_samples = ((*it_eventsL).timestamp.tm_min+(60 - prev_timestamp.tm_min));
				 }


				 //non dovrebbe entrarci mai?
				 if((*it_eventsL).timestamp.tm_hour > prev_timestamp.tm_hour+1){
					 int hours_counter2 = (*it_eventsL).timestamp.tm_hour - prev_timestamp.tm_hour ;
					 time_between_samples = hours_counter2*60+abs(((*it_eventsL).timestamp.tm_min-prev_timestamp.tm_min));
				 }
			 }

			 /*** GENERATION OF NEW SAMPLES ***/
			 if(flagstart==true){
				 time_between_samples=0;
			 }
			 else{
				 time_between_samples=1;
			 }

			 // Always on sensors: consider sampling frequency, take time between previous timestamp and current
			 // and generate a number of samples accordingly

			 float temp_value=fRand(1.0,5.0);
			 float temp_size=fRand(500.0, 2000.0); // Kilobyte
			 string temp_type="GPS";

			 // GPS
			 Location temp_loc(ref_lat,ref_lon,ref_alt);
			 Position temp_p(temp_loc,(*it_eventsL).timestamp);
			 Sample temp_sample(temp_type,temp_value,temp_size,false,temp_p);
			 temp_s.push_back(temp_sample);

			float tau_tx=lambda_g*(0.000192+((28.0+1500.0)/wifi_uplink_data_tx));// 192 us: PLCP time + (Header+Payload)/data rate
			float wifi_power=rho_id+rho_tx*tau_tx+lambda_g*gamma_g;// power to tx one packet

			// * * ACCELLEROMETER
			int acc_num_samples_to_generate=acc_sample_frequency*60*time_between_samples;
			int acc_data_size=(acc_num_samples_to_generate*acc_sample_size)/8;// to have Bytes
			int acc_energy_sampling=acc_current*(60*time_between_samples);// these are uAs (remember: 3600 uAs= 1uAh)
			float acc_energy_sampling_hour=acc_energy_sampling/3600000.00;// mAh

			int acc_packets=(int)acc_data_size/1500;
			float acc_tx_time=(acc_data_size*8.0)/wifi_uplink_data_tx;// in seconds (bit / bps)

			float acc_wifi_energy=wifi_power*acc_packets; // Joules (Watt*seconds)
			float acc_wifi_consumption=( acc_wifi_energy/( 1000*3.6 ) )*( 1000/volt ); // mah

			outputsimdata << currentDay << " "<<(*it_eventsL).timestamp.tm_hour << " "
						  << (*it_eventsL).timestamp.tm_min << " "
						  << sm.id_user << " "
						  << ref_lat << " "// position latitude
						  << ref_lon << " "// position longitude
						  << sm.battery << " "
						  << "Accelerometer" << " "
						  << acc_num_samples_to_generate << " "
						  << acc_data_size << " "// in Bytes
						  << acc_packets << " "// number of packets
						  << acc_tx_time << " "// uplink transmission time in seconds
						  << acc_energy_sampling_hour << " "// in uAh
						  << acc_wifi_energy//
						  << endl;

			// * * TEMPERATURE
			int gps_num_samples_to_generate=gps_sample_frequency*60*time_between_samples;
			int gps_data_size=(gps_num_samples_to_generate*gps_sample_size)/8;// to have Bytes
			int gps_energy_sampling=gps_current*(60*time_between_samples)*gps_sample_frequency;// these are uAs (remember: 3600 uAs= 1uAh)
			float gps_energy_sampling_hour=gps_energy_sampling/3600000.00;// uAh

			int gps_packets=(int)gps_data_size/1500;
			float gps_tx_time=(gps_data_size*8.0)/wifi_uplink_data_tx;// in seconds (Byte / Bps)
			float gps_wifi_energy=wifi_power*gps_packets; // Joules (Watt*seconds)

			float gps_wifi_consumption=( gps_wifi_energy/( 1000*3.6 ) )*( 1000/volt );

			outputsimdata << currentDay << " " << (*it_eventsL).timestamp.tm_hour << " "
						  << (*it_eventsL).timestamp.tm_min << " "
						  << sm.id_user << " "
						  << ref_lat << " "// position latitude
						  << ref_lon << " "// position longitude
						  << sm.battery << " "
						  << "GPS" << " "
						  << gps_num_samples_to_generate << " "
						  << gps_data_size << " "// in Bytes
						  << gps_packets << " "
						  << gps_tx_time << " "// uplink transmission time in seconds
						  << gps_energy_sampling_hour << " "// in mAh
						  << gps_wifi_energy
						  << endl;

			// * * PRESSURE
			int prox_num_samples_to_generate=prox_sample_frequency*60*time_between_samples;
			int prox_data_size=(prox_num_samples_to_generate*prox_sample_size)/8;// to have Bytes
			int prox_energy_sampling=prox_current*(60*time_between_samples);// these are uAs (remember: 3600 uAs= 1uAh)
			float prox_energy_sampling_hour=prox_energy_sampling/3600000.00;// mAh

			int prox_packets=(int)prox_data_size/1500;
			float prox_tx_time=(prox_data_size*8.0)/wifi_uplink_data_tx;// in seconds (Byte / Bps)
			float prox_wifi_energy=wifi_power*prox_packets; // Joules (Watt*seconds)
			float prox_wifi_consumption=( prox_wifi_energy/( 1000*3.6 ) )*( 1000/volt );

			outputsimdata << currentDay << " " << (*it_eventsL).timestamp.tm_hour << " "
						  << (*it_eventsL).timestamp.tm_min << " "
						  << sm.id_user << " "
						  << ref_lat << " "// position latitude
					      << ref_lon << " "// position longitude
						  << sm.battery << " "
						  << "Proximity" << " "
						  << prox_num_samples_to_generate << " "
						  << prox_data_size << " "// in Bytes
						  << prox_packets << " "
						  << prox_tx_time << " "//uplink transmission time in seconds
						  << prox_energy_sampling_hour << " "// in mAh
						  << prox_wifi_energy
						  << endl;


			 temp_dtbdelM["Accelerometer"]+=acc_data_size; // number of Bytes
			 temp_dtbdelM["GPS"]+=1.0;
			 float tot_data=0;

			 float cons_sens=0;
			 float cons_wifi=0;

			 if(sm.stop==0){
				 tot_data=acc_data_size+prox_data_size+gps_data_size;
				 temp_dtbdelM["data"]+=tot_data;
				 cons_sens=prox_energy_sampling_hour+gps_energy_sampling_hour+acc_energy_sampling_hour;
				 temp_dtbdelM["consumption-data"]+=cons_sens;

				 int packs=(int)tot_data/1500;
				 packs+=1;
				 if(typeCons==0)
					 packtotali +=packs;
				 float tot_wifi_energy=wifi_power*packs; // Joules (Watt*seconds)
				 cons_wifi=( tot_wifi_energy/( 1000*3.6 ) )*( 1000/volt );
				 temp_dtbdelM["consumption-wifi"]+=cons_wifi;
				 temp_dtbdelM["Energy"]+=cons_sens;
			}
/*			 cout<< acc_data_size<<" " << prox_data_size<<" "<< gps_data_size<<endl;

			 cout<<"DAI    " <<cons_wifi<<endl;*/



				 enTot= cons[typeCons]*temp_dtbdelM["Minutes"];

				 if(traces==false){
					 enTot=temp_dtbdelM["consumption-data"];
					 if(typeCons==0){
						 enTot+=temp_dtbdelM["consumption-wifi"];
						 temp_dtbdelM["Energy"]+=cons_wifi;
					 }
						 }

				 float enmax= (cap/100)*bat_threshold;




				 if(pda==true){
					 float si=datasent/dataexpected;
					 if(b>0 ){
						 if(si>upbound){
							 if(b>1)
								 b=b-1;

							 else
								 b=b/(1+b);

						 }
					 }
					if(b<50 ){
						 if(si<lowbound){
							 if(b<1)
								 b=b/(1-b);
							 else
								 b=b+1;
						 }
					 }
					 b=0.5;
					 threshold=1-pow(si,b);
					 //cout<< threshold<<" "<<datasent<<" "<<b<<" "<<si<<" "<<pow(si,b) <<endl;
				 }





				 if(typeCons==2){
					 float prob=(float(rand()) /float(RAND_MAX));
					 if(threshold<prob)
						 gen=false;
					 else
						 gen=true;

				 }

				 if(typeCons==3 && duration==0){
					 float prob=(float(rand()) /float(RAND_MAX));
					 if(percentages[((*it_eventsL).timestamp.tm_hour)]<prob)
						 gen=false;
					 else{
						 duration=duration_hour[((*it_eventsL).timestamp.tm_hour)];
						 gen=true;
					 }






				 }

				 if(duration>0)
					 duration-=time_between_samples;

				 if(typeCons==1)
					 gen=true;

				 if(typeCons==0){

					 if(enTot>=enmax && sm.stop==0){

						 sm.stop=1;


						 sb= sm.startBattery;
	/*					 if(typeCons==1){
							 ctx=temp_dtbdelM["Minutes"] * reporting_cost;

							 enTot+=ctx;



						 }*/

						old_battery=sm.battery;
						float rem= (cap/100)*old_battery;
						rem = rem - enTot;

						old_battery= (rem/cap)*100;
						locstop=make_pair(ref_lon,ref_lat);

						stopfile << ref_user <<" " << temp_dtbdelM["Total"] <<" "<< enTot <<" "<< sb-old_battery <<" "<<old_battery << " "<<(*(it_eventsL)).loc.lon<<" "<<(*(it_eventsL)).loc.lat<<endl;
						 //cout<< "Stopped"<<" "<<ref_user<<endl;
						gen=false;
					 }
					 if(sm.stop==0)
						 temp_dtbdelM["Minutes"]+=time_between_samples;
				 }


			 if(gen || typeCons!=0){
				 if(traces==true)
			 	 if(typeCons!=0)
				 temp_dtbdelM["Minutes"]+=time_between_samples;

				 temp_dtbdelM["Total"]+=(time_between_samples * bytes_per_minute);
				 if((typeCons==2 || typeCons==3) && gen ){
					 if(traces==true){
						 if(temp_dtbdelM["Total"]<kbpermin){
							 temp_dtsM["Total"]+=temp_dtbdelM["Total"];
							 datasent+=temp_dtbdelM["Total"];
							 temp_dtbdelM["Total"]=0;

						 }
						 else{
							 temp_dtsM["Total"]+=kbpermin;
							 temp_dtbdelM["Total"]-=kbpermin;
							 datasent+=kbpermin;
						 }
					 }
					 else{
						 temp_dtbdelM["Minutes"]+=time_between_samples;

						 if(temp_dtbdelM["data"]<kbpermin){

							 temp_dtsM["Total"]+=temp_dtbdelM["data"];

							 int tot_packets=(int)temp_dtbdelM["data"]/1500; //Bytes/dimension of one packet
							 tot_packets+=1;
							 float wifitemp=((tot_packets*wifi_power)/( 1000*3.6 ) )*( 1000/volt);
							 temp_dtbdelM["Energy"]+=wifitemp;
							 packtotali+=tot_packets;
							 datasent+=temp_dtbdelM["data"];
							 temp_dtbdelM["data"]=0;


						 }

						 else{

							 temp_dtsM["Total"]+=kbpermin;
							 temp_dtbdelM["data"]-=kbpermin;
							 int tot_packets=(int)kbpermin/1500;//Bytes/dimension of one packet
							 tot_packets+=1;
							 temp_dtbdelM["Energy"]+=((tot_packets*wifi_power)/( 1000*3.6 ) )*( 1000/volt);
							 datasent+=kbpermin;
						 }



					 }



				 }
			 }
//			 if(ref_user==42)
//				 cout<< ref_lon<<" "<<ref_lat<<endl;
			 route.push_back(make_pair(ref_lon,ref_lat));
			 generate.push_back(gen);
			 if(gen==true && flagstart==false)
				 numgeneration+=1;





			 if(flagstart)
				 flagstart=false;


		 }


		 cout <<"Tot Data Generated in all the days:	"<<datatotalitotgiorni<<"  KB"<<endl;
		 fairfile <<"tot Contribution  "<<totcontribution<<" quad contrib  "<<quadcontribution<<" Fairness  "<<((totcontribution*totcontribution)/quadcontribution)/num_users<<endl;
		
		 batfairfile <<"BATTERY CONTRIBUTION  "<<batcontrib<<" quad contrib  "<<quadbatcontrib<<" Fairness  "<<((batcontrib*batcontrib)/quadbatcontrib)/num_users<<endl;



		 int userusati2=num_users;
		 results.push_back(datatotali/userusati2);
		 results.push_back(cons_sensing/userusati2);
		 results.push_back(cons_report/userusati2);
		 cons_report=0;
		 cons_sensing=0;
		 datatotali=0;

		 counteruser=ref_user;

		perc.close();
	    outputsimdata.close();
	    stopfile.close();
	    activefile.close();

	    return results;
}

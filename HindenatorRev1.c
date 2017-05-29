/*HINDENATOR CODE REVISION 1
05/02/16
*/

//task definitions
task yaw_left();
task yaw_right();
task pitch_up();
task pitch_down();
task throttle_up();
task throttle_down();
task wall_avoid();
task ground_avoid();
task center_controls();
task get_mail();

//global mail variable
int mail=0;

//abstracted fucntion definitions
void deflect_elevator(int direction);
void deflect_rudder(int direction);
void incriment_throttle(int amount);

task main() {
	nxtDisplayTextLine(0,"waiting for commands...");
	//initialising threads
	startTask(get_mail);
	startTask(yaw_left);
	startTask(yaw_right);
	startTask(pitch_up);
	startTask(pitch_down);
	startTask(throttle_up);
	startTask(throttle_down);
	startTask(wall_avoid);
	startTask(ground_avoid);
	startTask(center_controls);
	//keeps threads alive
	while (true) {
		wait1Msec(30000);
	}

}

task yaw_left() {
	//MailBox Numer:51
	while (true) {
		if(mail==51)
		{
			nxtDisplayTextLine(0,"yawing left");
			deflect_rudder(1);
		}


	}
}

task yaw_right() {
	//MailBox Numer:53
	while (true) {
		if(mail==53)
		{
			nxtDisplayTextLine(0,"yawing right");
			deflect_rudder(-1);
		}


	}
}

task pitch_up() {
	//Mailbox Number: 55
	while (true) {
		if(mail==55)
		{
			nxtDisplayTextLine(0,"pitching up");
			deflect_elevator(1);
		}


	}
}

task pitch_down() {
	//Mailbox Number:49
	while (true) {
		if(mail==49)
		{
			nxtDisplayTextLine(0,"pitching down");
			deflect_elevator(1);
		}


	}
}

task throttle_up() {
	//MailBox Number: 54
	while (true) {
		if(mail==54)
		{
			nxtDisplayTextLine(0,"throttling up");
			incriment_throttle(1);
		}


	}
}

task throttle_down() {
	//MailBox Number: 57
	while (true) {
		if(mail==57)
		{
			nxtDisplayTextLine(0,"throttling down");
			incriment_throttle(-1);
		}


	}
}
task center_controls() {
	//MailBox Number: 52
	while (true) {
		if(mail==52)
		{
			nxtDisplayTextLine(0,"centering controlls");
			//INSERT CONTROLL CODE HERE
		}


	}
}

task wall_avoid() {
	while (true) {
		//INSERT CONTROLL CODE HERE

	}
}

task ground_avoid() {
	while (true) {
		//INSERT CONTROLL CODE HERE
	}
}

task get_mail() {
	while(true)
	{
		mail= message;
		wait10Msec(1);
		ClearMessage();
	}
}
void deflect_elevator(int direction)
{
	//INSERT CONTROLL CODE HERE
}
void deflect_rudder(int direction)
{
	//INSERT CONTROLL CODE HERE
}
void incriment_throttle(int amount)
{
	//INSERT CONTROLL CODE HERE
}

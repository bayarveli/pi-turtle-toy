/*
 * GpioPin.cpp
 *
 *  Created on: Mar 18, 2015
 *      Author: makcakoca
 */
//!  A test class.
/*!
  A more elaborate class description.
*/

#include "gpio_pin.h"

using namespace std;

bool g_b_exit_signal = false;


const string GpioPin::RISING = "rising";
const string GpioPin::FALLING = "falling";
const string GpioPin::BOTH = "both";

const string GpioPin::INPUT = "in";
const string GpioPin::OUTPUT = "out";

const string GpioPin::HIGH = "1";
const string GpioPin::LOW = "0";

const string GpioPin::GPIO0 = "23";
const string GpioPin::GPIO1 = "24";
const string GpioPin::GPIO2 = "25";
const string GpioPin::GPIO3 = "7";
const string GpioPin::GPIO4 = "1";
const string GpioPin::GPIO5 = "16";
const string GpioPin::GPIO6 = "20";
const string GpioPin::GPIO7 = "21";
const string GpioPin::GPIO8 = "26";
const string GpioPin::GPIO9 = "6";
const string GpioPin::GPIO10 = "5";
const string GpioPin::GPIO11 = "0";
const string GpioPin::GPIO12 = "12";
const string GpioPin::GPIO13 = "19";


GpioPin::GpioPin():i_value_fd(-1),i_direction_fd(-1),i_export_fd(-1),
i_unexport_fd(-1),i_edge_fd(-1),str_pin_number("23")
{
//	b_exit_signal = false;
//	signal(SIGINT, CatchSignal);
//	str_interrupt_mode = "";
	this->ExportGpio();
}

/*!
	\param _str_pin_number GPIO pin numarasi.
*/
GpioPin::GpioPin(string _str_pin_number):i_value_fd(-1),
i_direction_fd(-1),i_export_fd(-1),i_unexport_fd(-1),i_edge_fd(-1),
str_pin_number(_str_pin_number)
{
//	b_exit_signal = false;
//	signal(SIGINT, CatchSignal);
//	str_interrupt_mode = "";
	this->ExportGpio();
}


GpioPin::~GpioPin()
{
	printf("gpio kapatiliyor.\n");
	this->UnexportGpio();
}

/*!
	\return Fonksiyon durum bilgisi donmektedir.
	Negatif donusler islem sirasinda hata gerlestigini gostermektedir.
*/
int GpioPin::ExportGpio()
{
	int i_status_value = -1;
	string str_export = "/sys/class/gpio/export";

	this->i_export_fd = i_status_value = open(str_export.c_str(),
												O_WRONLY|O_SYNC);
	// HATA01
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO export aygiti acilamadi.");
        exit(1);
	}

	printf("length: %d, sizeof: %d\n", this->str_pin_number.length(), sizeof(this->str_pin_number.c_str()));

	i_status_value = write(this->i_export_fd, this->str_pin_number.c_str(),
								this->str_pin_number.length());
	// HATA02
	if (i_status_value < 0)
	{
		if (errno == EBUSY) {
			printf("GPIO %s is already exported - continuing.\n", this->str_pin_number.c_str());
			close(this->i_export_fd);
			return 0; // success
		}
		perror("GpioPin: SYSFS GPIO export aygitina yazarken hata olustu.");
        	exit(1);
	}

	i_status_value = close(this->i_export_fd);
	// HATA03
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO export aygiti kapatilamadi.");
        exit(1);
	}

    return i_status_value;
}

/*!
	\return Fonksiyon durum bilgisi donmektedir.
	Negatif donusler islem sirasinda hata gerlestigini gostermektedir.
*/
int GpioPin::UnexportGpio()
{
	int i_status_value = -1;
	string str_unexport = "/sys/class/gpio/unexport";

	this->i_export_fd = i_status_value = open(str_unexport.c_str(),
												O_WRONLY|O_SYNC);

	// HATA04
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO unexport aygiti acilamadi");
        exit(1);
	}


	i_status_value = write(this->i_export_fd, this->str_pin_number.c_str(),
						this->str_pin_number.length());

	// HATA05
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO unexport aygitina yazarken hata oluştu.");
        exit(1);
	}

	i_status_value = close(this->i_export_fd);
	// HATA06
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO unexport aygiti kapatilamadi.");
        exit(1);
	}

	return i_status_value;
}

/*!
	\param str_direction Pin icin atanacak mod tipidir.
	Pini girdi olarak kullanmak icin INPUT, cikti olarak kullanmak icin
	ise OUTPUT argumani girilmelidir.
	\return Fonksiyon durum bilgisi donmektedir.
	Negatif donusler islem sirasinda hata gerlestigini gostermektedir.
*/
int GpioPin::SetPinDirection(string str_direction)
{
	int i_status_value = -1;
	string str_setdir ="/sys/class/gpio/gpio" + this->str_pin_number + "/direction";


	this->i_direction_fd = i_status_value = open(str_setdir.c_str(),
												O_WRONLY|O_SYNC);
	// HATA07
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO direction aygiti acilamadi.");
		exit(1);
	}

	// HATA08
	if (str_direction.compare(INPUT) != 0 && str_direction.compare(OUTPUT) != 0 )
	{
		fprintf(stderr, "GpioPin: Gecersiz Pin Modu Atamasi.\n INPUT ya da OUTPUT olmali.\n");
		exit(1);
	}

	i_status_value = write(this->i_direction_fd, str_direction.c_str(),
							str_direction.length());
	// HATA09
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO direction aygitina yazilamadi.");
        exit(1);
	}

	i_status_value = close(this->i_direction_fd);
	// HATA10
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO direction aygiti kapatilamadi.");
        exit(1);
	}

	    return i_status_value;
}

/*!
	\param str_value Pin icin atanacak degerdir.
	1 icin HIGH, 0 icin LOW girdisi verilmelidir.
	\return Fonksiyon durum bilgisi donmektedir.
	Negatif donusler islem sirasinda hata gerlestigini gostermektedir.
*/
int GpioPin::SetPinValue(string str_value)
{

    int i_status_value = -1;
	string str_set_value = "/sys/class/gpio/gpio" + this->str_pin_number + "/value";

	this->i_value_fd = i_status_value = open(str_set_value.c_str(),
												O_WRONLY|O_SYNC);
	// HATA11
	if (i_status_value < 0)
	{
			perror("GpioPin: SYSFS GPIO value aygiti acilamadi.");
        exit(1);
	}

	// HATA12
	if (str_value.compare(HIGH) != 0 && str_value.compare(LOW) != 0 )
	{
		fprintf(stderr, "GpioPin: Gecersiz deger girildi. HIGH ya da LOW olmali \n");
		exit(1);
	}

	i_status_value = write(this->i_value_fd, str_value.c_str(),
							str_value.length());
	// HATA13
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO value aygitina yazilamadi.");
        exit(1);
	}

	i_status_value = close(this->i_value_fd);
	// HATA14
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO value aygiti kapatilamadi.");
        exit(1);
	}
	return i_status_value;
}

/*!
	\param &str_value pinin degeri bu arguman uzerinden donecektir.
	\return Fonksiyon durum bilgisi donmektedir.
	Negatif donusler islem sirasinda hata gerlestigini gostermektedir.
*/
int GpioPin::GetPinValue(string & str_value)
{

	string str_get_value = "/sys/class/gpio/gpio" + this->str_pin_number + "/value";

	char c_buff[2];
	int i_status_value = -1;
	this->i_value_fd = i_status_value = open(str_get_value.c_str(),
											O_RDONLY|O_SYNC);
	// HATA11
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO value aygiti acilamadi");
        exit(1);
	}

	i_status_value = read(this->i_value_fd, &c_buff, 1);
	// HATA15
	if (i_status_value < 0){
		perror("GpioPin: SYSFS GPIO value aygiti okunamadi");
        exit(1);
	}

	c_buff[1]='\0';

	str_value = string(c_buff);

	if (str_value.compare(HIGH) != 0 && str_value.compare(LOW) != 0 ) {
		fprintf(stderr, "GpioPin: Gecersiz deger okundu. HIGH ya da LOW olmali. \n");
		exit(1);
	}

	i_status_value = close(this->i_value_fd);
	// HATA14
	if (i_status_value < 0){
		perror("GpioPin: SYSFS GPIO value aygiti kapatilamadi.");
        exit(1);
	}

	return i_status_value;
}

/*!
	\return Pin numarasi donmektedir.
*/
string GpioPin::GetPinNumber() const
{
	return this->str_pin_number;
}

/*!
    \param _callback_function Callback fonksiyonu
	\param void *p Callback fonksiyonunun bulundugu sınıfın nesnesini isaret etmektedir.
*/
/*void GpioPin::SetCallback(CallbackFunctionPtr _callback_function, void *p)
{
	callback_function = _callback_function;
	p_v_callback_pointer = p;
}*/


/*int GpioPin::SetInterruptMode(string str_int_mode)
{

	int i_status_value = -1;
	string str_set_edge = "/sys/class/gpio/gpio" + this->str_pin_number + "/edge";

	// HATA16
	if(str_int_mode.compare(RISING) != 0 && str_int_mode.compare(FALLING) != 0
		&& str_int_mode.compare(BOTH) != 0)
	{
		fprintf(stderr, "GpioPin: Gecersiz interrupt modu. RISING, FALLING ya da BOTH olmali\n");
		exit(1);
	}

	this->str_interrupt_mode = str_int_mode;

	this->i_edge_fd = i_status_value = open(str_set_edge.c_str(),
												O_WRONLY|O_SYNC);
	// HATA17
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO edge aygiti acilamadi.");
        exit(1);
	}

	i_status_value = write(this->i_edge_fd, str_int_mode.c_str(),
						str_int_mode.length());
	// HATA18
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO edge aygitina yazilamadi.");
        exit(1);
	}

	i_status_value = this->PollGpio();

	i_status_value = close(this->i_edge_fd);

	// HATA19
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO edge aygiti kapatilamadi.");
        exit(1);
	}
	return i_status_value;

}
*/
/*
int GpioPin::PollGpio()
{

	string str_get_value = "/sys/class/gpio/gpio" + this->str_pin_number + "/value";

	int i_status_value = -1;
	this->i_value_fd = i_status_value = open(str_get_value.c_str(),
											O_RDONLY|O_SYNC);
	// HATA11
	if (i_status_value < 0)
	{
		perror("GpioPin: SYSFS GPIO value aygiti acilamadi");
        exit(1);
	}

	if(str_interrupt_mode.compare(RISING) != 0 && str_interrupt_mode.compare(FALLING) != 0
		&& str_interrupt_mode.compare(BOTH) != 0)
	{
		fprintf(stderr, "GpioPin: Interrupt modu atanmamis.\n");
		exit(1);
	}

	//while(g_b_exit_signal == false)
	//{

		if (-1 != i_value_fd)
		{
			struct pollfd poll_gpio;

			poll_gpio.fd = i_value_fd;
			poll_gpio.events = POLLPRI|POLLERR;

			int rv = poll(&poll_gpio, 1, -1);    // block endlessly
			if (rv > 0)
			{
				if (poll_gpio.revents & POLLPRI)
				{
					// IRQ happened
					char buf[2];
					lseek(poll_gpio.fd, 0, SEEK_SET);
					// int n = read(poll_123.fd, buf, sizeof(buf));

					int n = pread(poll_gpio.fd, buf, sizeof(buf), 0);

					if (n > 0)
					{
						int gpio_val = (buf[0] == 48) ? 0 : 1;

						if(gpio_val == 0 && str_interrupt_mode.compare(FALLING) == 0)
						{
							cout << "FALLING INTERRUPT" << endl;
							//callback_function(p_v_callback_pointer, gpio_val);
						}
						else if(gpio_val == 1 && str_interrupt_mode.compare(RISING) == 0)
						{
							cout << "RISING INTERRUPT" << endl;
							//callback_function(p_v_callback_pointer, gpio_val);
						}
						else if(str_interrupt_mode.compare(BOTH) == 0)
						{
							cout << "BOTH INTERRUPT" << endl;
							//callback_function(p_v_callback_pointer, gpio_val);
						}
					}
				}
			}
		}
	//}

	return i_status_value;
}
*/
/*void CatchSignal(int i_sig)
{
        signal(SIGINT, CatchSignal);
        g_b_exit_signal = true;
        printf("Sinyal yakalandi...\n");
}*/




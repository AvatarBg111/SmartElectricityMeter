#include "electrical_consumers.h"
#include "main.h"

void set_id_consumer(){
	if(event_control.CURRENT_RMS > 750 && event_control.CURRENT_RMS < 1000){
		event_control.id_consumer = CONSUMER_LAMP;
	} else if(event_control.CURRENT_RMS > 1590 && event_control.CURRENT_RMS < 1610){
		event_control.id_consumer = CONSUMER_IRON;
	} else {
		event_control.id_consumer = 0;
	}
}
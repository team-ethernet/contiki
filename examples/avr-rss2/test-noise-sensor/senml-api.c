struct senML_pack{
	void * first_record = NULL
	void * last_record = NULL
}

// Flags are used to keep track of which values has been assigned.
struct base_record
{
    void * next_record = NULL
	char * base_name = NULL
	double base_time = 0
	char * base_unit = NULL
	double base_value = 0
	double base_sum = 0
	double base_version = 0
	uint16_t flags = 0
}

// Flags are used to keep track of which values has been assigned.
struct regular_record
{
	void * next_record = NULL
	char * name = NULL
	char * unit = NULL
	double value = 0
	char * string_value = NULL
	char boolean_value
	char * data_value = NULL
	double sum = 0
	double time = 0
	double update_time
    uint16_t flags = 0
}

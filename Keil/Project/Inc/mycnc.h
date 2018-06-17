#pragma once
//stepper object that controls the movement of a stepper
class Stepper
{

public:
	/**
	 * Constructor
	 */
	Stepper();
	//initialize ID to this
	Stepper (int _ID);//
	/**
	 * Destructor, also kills the mavlinks task.
	 **************
	 This might be problematic
	 */
	~Stepper() {};
	void set_dir(int dir);
	void one_step(int duration);
	void simple_step(int num,int dir,int duration);
	void get_ID() {cout<<"rua";};
	void trigger();
protected:
	
private:
	int stepper_id;
	//port needed as well
	int dir_pin;
	//maybe not needed, because the step pin is tied to the timer
	int step_pin;
};


//stepper object that describe one axis of the cnc

class CNC_Axis
{

public:
	/**
	 * Constructor
	 */
	CNC_Axis();

	/**
	 * Destructor, also kills the mavlinks task.
	 */
	~CNC_Axis();

	/**
	* Start the mavlink task.
	 *
	 * @return		OK on success.
	 */
	void calibration(void);
	void go_to_step(int target_step);

	static int		start(int argc, char *argv[]);

	/**
	 * Display the mavlink status.
	 */
	void			display_status();

	static int		stream_command(int argc, char *argv[]);

protected:
	Mavlink			*next;

private:
	int			_instance_id;
	bool			_transmitting_enabled;
	bool			_transmitting_enabled_commanded;

	orb_advert_t		_mavlink_log_pub;
	bool			_task_running;
	static bool		_boot_complete;
	static constexpr unsigned MAVLINK_MAX_INSTANCES = 4;
	static constexpr unsigned MAVLINK_MIN_INTERVAL = 1500;
	static constexpr unsigned MAVLINK_MAX_INTERVAL = 10000;
	static constexpr float MAVLINK_MIN_MULTIPLIER = 0.0005f;
	mavlink_message_t _mavlink_buffer;
	mavlink_status_t _mavlink_status;

	/* states */
	bool			_hil_enabled;		/**< Hardware In the Loop mode */
	bool			_generate_rc;		/**< Generate RC messages from manual input MAVLink messages */
	bool			_use_hil_gps;		/**< Accept GPS HIL messages (for example from an external motion capturing system to fake indoor gps) */
	bool			_forward_externalsp;	/**< Forward external setpoint messages to controllers directly if in offboard mode */
	bool			_is_usb_uart;		/**< Port is USB */
	bool			_wait_to_transmit;  	/**< Wait to transmit until received messages. */
	bool			_received_messages;	/**< Whether we've received valid mavlink messages. */

	unsigned		_main_loop_delay;	/**< mainloop delay, depends on data rate */

	MavlinkOrbSubscription	*_subscriptions;
	MavlinkStream		*_streams;

	MavlinkShell			*_mavlink_shell;
	MavlinkULog			*_mavlink_ulog;
	volatile bool			_mavlink_ulog_stop_requested;

	MAVLINK_MODE 		_mode;

	mavlink_channel_t	_channel;
	int32_t			_radio_id;

	ringbuffer::RingBuffer		_logbuffer;

	pthread_t		_receive_thread;

	bool			_forwarding_on;
	bool			_ftp_on;
#ifndef __PX4_QURT
	int			_uart_fd;
#endif
	int			_baudrate;
	int			_datarate;		///< data rate for normal streams (attitude, position, etc.)
	int			_datarate_events;	///< data rate for params, waypoints, text messages
	float			_rate_mult;
	hrt_abstime		_last_hw_rate_timestamp;

	/**
	 * If the queue index is not at 0, the queue sending
	 * logic will send parameters from the current index
	 * to len - 1, the end of the param list.
	 */
	unsigned int		_mavlink_param_queue_index;

	bool			mavlink_link_termination_allowed;

	char 			*_subscribe_to_stream;
	float			_subscribe_to_stream_rate;
	bool 			_udp_initialised;

	enum FLOW_CONTROL_MODE	_flow_control_mode;
	uint64_t		_last_write_success_time;
	uint64_t		_last_write_try_time;
	uint64_t		_mavlink_start_time;
	int32_t			_protocol_version_switch;
	int32_t			_protocol_version;

	unsigned		_bytes_tx;
	unsigned		_bytes_txerr;
	unsigned		_bytes_rx;
	uint64_t		_bytes_timestamp;
	float			_rate_tx;
	float			_rate_txerr;
	float			_rate_rx;

#ifdef __PX4_POSIX
	struct sockaddr_in _myaddr;
	struct sockaddr_in _src_addr;
	struct sockaddr_in _bcast_addr;
	bool _src_addr_initialized;
	bool _broadcast_address_found;
	bool _broadcast_address_not_found_warned;
	bool _broadcast_failed_warned;
	uint8_t _network_buf[MAVLINK_MAX_PACKET_LEN];
	unsigned _network_buf_len;
#endif
	int _socket_fd;
	Protocol	_protocol;
	unsigned short _network_port;
	unsigned short _remote_port;

	struct telemetry_status_s	_rstatus;			///< receive status

	struct ping_statistics_s	_ping_stats;		///< ping statistics

	struct mavlink_message_buffer {
		int write_ptr;
		int read_ptr;
		int size;
		char *data;
	};

	mavlink_message_buffer	_message_buffer;

	pthread_mutex_t		_message_buffer_mutex;
	pthread_mutex_t		_send_mutex;

	bool			_param_initialized;
	int32_t			_broadcast_mode;

	param_t			_param_system_id;
	param_t			_param_component_id;
	param_t			_param_proto_ver;
	param_t			_param_radio_id;
	param_t			_param_system_type;
	param_t			_param_use_hil_gps;
	param_t			_param_forward_externalsp;
	param_t			_param_broadcast;

	unsigned		_system_type;
	static bool		_config_link_on;

	perf_counter_t		_loop_perf;			/**< loop performance counter */
	perf_counter_t		_txerr_perf;			/**< TX error counter */

	void			mavlink_update_system();

#ifndef __PX4_QURT
	int			mavlink_open_uart(int baudrate, const char *uart_name, bool force_flow_control);
#endif

	static int		interval_from_rate(float rate);

	static constexpr unsigned RADIO_BUFFER_CRITICAL_LOW_PERCENTAGE = 25;
	static constexpr unsigned RADIO_BUFFER_LOW_PERCENTAGE = 35;
	static constexpr unsigned RADIO_BUFFER_HALF_PERCENTAGE = 50;

	int configure_stream(const char *stream_name, const float rate = -1.0f);

	/**
	 * Adjust the stream rates based on the current rate
	 *
	 * @param multiplier if greater than 1, the transmission rate will increase, if smaller than one decrease
	 */
	void adjust_stream_rates(const float multiplier);

	int message_buffer_init(int size);

	void message_buffer_destroy();

	int message_buffer_count();

	int message_buffer_is_empty();

	int message_buffer_get_ptr(void **ptr, bool *is_part);

	void message_buffer_mark_read(int n);

	void pass_message(const mavlink_message_t *msg);

	/**
	 * Check the configuration of a connected radio
	 *
	 * This convenience function allows to re-configure a connected
	 * radio without removing it from the main system harness.
	 */
	void check_radio_config();

	/**
	 * Update rate mult so total bitrate will be equal to _datarate.
	 */
	void update_rate_mult();

	void find_broadcast_address();

	void init_udp();

	/**
	 * Main mavlink task.
	 */
	int		task_main(int argc, char *argv[]);

	/* do not allow copying this class */
	Mavlink(const Mavlink &);
	Mavlink operator=(const Mavlink &);
};
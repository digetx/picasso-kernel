
#ifndef __MFD_EC_CONTROL_H
#define __MFD_EC_CONTROL_H

struct ec_reg_data {
	u8  addr;
	u16 timeout;
};

#define EC_REG_DATA(_name, _addr, _timeout)		\
const static struct ec_reg_data ec_##_name##_ = {	\
	.addr = _addr,					\
	.timeout = _timeout,				\
};							\
const static struct ec_reg_data *_name = &ec_##_name##_;

int ec_read_word_data_locked(const struct ec_reg_data *reg_data);
int ec_read_word_data(const struct ec_reg_data *reg_data);
int ec_write_word_data_locked(const struct ec_reg_data *reg_data, u16 value);
int ec_write_word_data(const struct ec_reg_data *reg_data, u16 value);
void ec_lock(void);
void ec_unlock(void);
int get_board_id(void);

#endif	/* __MFD_EC_CONTROL_H */

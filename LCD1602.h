/*
 * LCD1602.h
 *
 *  Created on: Jul 2, 2024
 *      Author: Krushna
 */
#ifndef INC_LCD1602_H_
#define INC_LCD1602_H_


void lcd_init(void);
void lcd_send_cmd(char cmd);
void lcd_send_data(char data);
void lcd_send_string(char *str);
void lcd_put_cur(int row,int col);
void lcd_clear(void);



#endif /* INC_LCD1602_H_ */


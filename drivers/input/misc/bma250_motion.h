/*
 *
 * Bosh BMA 250. Digital, triaxial acceleration sensor.
 *
 */
#ifndef BMA250_MOTION_MODULE_H
#define BMA250_MOTION_MODULE_H

extern int bma250_ic_read(struct i2c_client *ic_dev, u8 reg, u8 *buf, int len);
extern int bma250_ic_write(struct i2c_client *ic_dev, u8 reg, u8 val);

extern void * __devinit bma250_motion_probe(struct i2c_client *ic_dev);
extern int __devexit bma250_motion_remove(void *dev);

#if defined(CONFIG_PM)
extern int bma250_motion_suspend(void *dev);
extern int bma250_motion_resume(void *dev);
#endif /* CONFIG_PM */

#endif /* BMA250_MOTION_MODULE_H */

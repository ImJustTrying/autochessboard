use gpio_cdev::{Chip, LineRequestFlags};
use std::{thread, time};
const PIN: u32 = 17;

fn main() -> Result<(), gpio_cdev::Error> {
    let mut chip = Chip::new("/dev/gpiochip0")?;
    println!("Chip name: {}, chip label: {}", chip.name(), chip.label());
    let output = chip.get_line(PIN)?;
    let output_handle = output.request(LineRequestFlags::OUTPUT, 0, "LED")?;
    output_handle.set_value(1)?;
    thread::sleep(time::Duration::from_secs(3));
    output_handle.set_value(0)?;
    Ok(())
}

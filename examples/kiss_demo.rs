use hex_slice::AsHex;
use kiss3d::light::Light;
use kiss3d::nalgebra::Point3;
use kiss3d::window::Window;
use rplidar_driver::rpos_drv::RposError;
use rplidar_driver::utils::sort_scan;
use rplidar_driver::ScanOptions;
use rplidar_driver::{Health, RplidarDevice};
use std::env;

fn main() {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 || args.len() > 3 {
        println!("Usage: {} <serial_port> ", args[0]);
        return;
    }

    let port_name = &args[1];

    let mut rplidar = RplidarDevice::open_port(&port_name).unwrap();

    let device_info = rplidar
        .get_device_info()
        .expect("failed to get device info");

    println!("Connected to LIDAR: ");
    println!("    Model: {}", device_info.model);
    println!(
        "    Firmware Version: {}.{}",
        device_info.firmware_version >> 8,
        device_info.firmware_version & 0xff
    );
    println!("    Hardware Version: {}", device_info.hardware_version);
    println!(
        "    Serial Number: {:02X}",
        device_info.serialnum.plain_hex(false)
    );

    let device_health = rplidar
        .get_device_health()
        .expect("failed to get device health");

    match device_health {
        Health::Healthy => {
            println!("LIDAR is healthy.");
        }
        Health::Warning(error_code) => {
            println!("LIDAR is unhealthy, warn: {:04X}", error_code);
        }
        Health::Error(error_code) => {
            println!("LIDAR is unhealthy, error: {:04X}", error_code);
        }
    }

    let all_supported_scan_modes = rplidar
        .get_all_supported_scan_modes()
        .expect("failed to get all supported scan modes");

    println!("All supported scan modes:");
    for scan_mode in all_supported_scan_modes {
        println!(
            "    {:2} {:16}: Max Distance: {:6.2}m, Ans Type: {:02X}, Us per sample: {:.2}us",
            scan_mode.id,
            scan_mode.name,
            scan_mode.max_distance,
            scan_mode.ans_type,
            scan_mode.us_per_sample
        );
    }

    let typical_scan_mode = rplidar
        .get_typical_scan_mode()
        .expect("failed to get typical scan mode");

    println!("Typical scan mode: {}", typical_scan_mode);

    match rplidar.check_motor_ctrl_support() {
        Ok(support) if support => {
            println!("Accessory board is detected and support motor control, starting motor...");
            rplidar.set_motor_pwm(600).expect("failed to start motor");
        }
        Ok(_) => {
            println!("Accessory board is detected, but doesn't support motor control");
        }
        Err(_) => {
            println!("Accessory board isn't detected");
        }
    }

    println!("Starting LIDAR in typical mode...");

    let scan_options = ScanOptions::with_mode(2);

    let actual_mode = rplidar
        .start_scan_with_options(&scan_options)
        .expect("failed to start scan in standard mode");

    println!("Started scan in mode `{}`", actual_mode.name);

    let start_time = std::time::Instant::now();

    let mut window = Window::new("Kiss3d: points");

    window.set_light(Light::StickToCamera);
    window.set_point_size(10.0);
    window.set_background_color(0.0, 0.0, 0.0);

    loop {
        match rplidar.grab_scan() {
            Ok(mut scan) => {
                println!(
                    "[{:6}s] {} points per scan",
                    start_time.elapsed().as_secs(),
                    scan.len()
                );

                sort_scan(&mut scan).unwrap();

                let scan = scan
                    .into_iter()
                    .filter(|scan| scan.is_valid())
                    .collect::<Vec<_>>();

                let points = scan
                    .into_iter()
                    .map(|scan_point| {
                        let x = scan_point.distance() * (-scan_point.angle()).cos();
                        let y = scan_point.distance() * (-scan_point.angle()).sin();
                        Point3::new(x, y, 0.0).yzx()
                    })
                    .collect::<Vec<_>>();

                for point in points {
                    window.draw_point(&point, &Point3::new(1.0, 0.0, 0.0));
                }
                window.draw_point(&Point3::new(0.0, 0.0, 0.0), &Point3::new(0.0, 1.0, 0.0));
                window.draw_line(
                    &Point3::new(0.0, 0.0, 0.0),
                    &Point3::new(1.0, 0.0, 0.0).yzx(),
                    &Point3::new(0.0, 1.0, 0.0),
                );
                if !window.render() {
                    break;
                }
            }
            Err(err) => {
                if let Some(RposError::OperationTimeout) = err.downcast_ref::<RposError>() {
                    continue;
                } else {
                    println!("Error: {:?}", err);
                    break;
                }
            }
        }
    }

    drop(window);
    rplidar.stop_motor().unwrap();
    println!("Lidar stopped\nPress enter to exit...");
    std::io::stdin().read_line(&mut String::new()).unwrap();
}

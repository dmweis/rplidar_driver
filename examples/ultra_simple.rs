extern crate hex_slice;
extern crate rplidar_drv;

use hex_slice::AsHex;
use rplidar_drv::rpos_drv::RposError;
use rplidar_drv::utils::sort_scan;
use rplidar_drv::ScanOptions;
use rplidar_drv::{Health, RplidarDevice};
use std::time::{Duration, Instant};

use std::env;

use plotters::prelude::*;

fn main() {
    let args: Vec<String> = env::args().collect();

    if args.len() < 2 || args.len() > 3 {
        println!("Usage: {} <serial_port>", args[0]);
        return;
    }

    let serial_port = &args[1];

    let mut rplidar = RplidarDevice::open_port(&serial_port).unwrap();

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

    let mut last_plot = Instant::now();

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

                if last_plot.elapsed() > Duration::from_secs_f32(2.0) {
                    last_plot = Instant::now();
                    let root = BitMapBackend::new("test_plot.png", (640, 480)).into_drawing_area();
                    root.fill(&WHITE).unwrap();
                    let mut chart = ChartBuilder::on(&root)
                        .caption("y=x^2", ("sans-serif", 50).into_font())
                        .margin(5)
                        .x_label_area_size(30)
                        .y_label_area_size(30)
                        .build_cartesian_2d(0f32..360f32, 0f32..12f32)
                        .unwrap();

                    chart.configure_mesh().draw().unwrap();

                    chart
                        .draw_series(LineSeries::new(
                            scan.into_iter()
                                .map(|point| (point.angle().to_degrees(), point.distance())),
                            &RED,
                        ))
                        .unwrap()
                        .label("laser scan")
                        .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));

                    chart
                        .configure_series_labels()
                        .background_style(&WHITE.mix(0.8))
                        .border_style(&BLACK)
                        .draw()
                        .unwrap();
                }

                // for scan_point in scan {
                //     println!(
                //         "    Angle: {:5.2}, Distance: {:8.4}, Valid: {:5}, Sync: {:5}",
                //         scan_point.angle().to_degrees(),
                //         scan_point.distance(),
                //         scan_point.is_valid(),
                //         scan_point.is_sync()
                //     )
                // }
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
}

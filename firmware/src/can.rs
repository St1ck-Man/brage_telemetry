use embassy_stm32::can::frame::{FdFrame, Header};

use crate::{protocol, CAN_IN, CAN_OUT};

#[embassy_executor::task]
pub async fn can_reader_task(can_rx: embassy_stm32::can::BufferedFdCanReceiver) -> ! {
    loop {
        let res = can_rx.receive().await.unwrap();

        let id: u16 = if let embedded_can::Id::Standard(can_id) = res.frame.id() {
            can_id.as_raw()
        } else {
            panic!("Does not support extended IDs")
        };
        let data = res.frame.data();

        let mut msg = protocol::CanMsg {
            id: id,
            length: data.len() as u8,
            data: [0; 64],
        };
        msg.data[0..data.len()].copy_from_slice(&data[0..data.len()]);
        CAN_IN.send(msg).await;
    }
}

#[embassy_executor::task]
pub async fn can_writer_task(mut can_tx: embassy_stm32::can::BufferedFdCanSender) -> ! {
    loop {
        let msg = CAN_OUT.receive().await;
        let can_id = embedded_can::StandardId::new(msg.id).expect("can ID failed lol");

        let header: Header = Header::new(can_id.into(), msg.length, false);
        let frame: FdFrame = FdFrame::new(header, &msg.data[0..msg.length as usize]).unwrap();
        can_tx.write(frame).await;
    }
}

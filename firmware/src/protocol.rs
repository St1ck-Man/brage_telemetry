pub struct CanMsg {
    pub id: u16,
    pub length: u8,
    pub data: [u8; 64],
}

pub struct RangingMsg {
    pub lon: u32,
    pub lat: u32,
    pub distance_cm: u32,
}

pub enum Msg {
    Can(CanMsg),
}

const CAN_FRAME_OVERHEAD: usize = 4;
const HEADER_SIZE: usize = 4;
const FOOTER_SIZE: usize = 2;

const X25: crc::Crc<u16> = crc::Crc::<u16>::new(&crc::CRC_16_IBM_SDLC);

pub fn begin_buffer(data: &mut [u8], id: u8) -> Result<usize, usize> {
    if data.len() < HEADER_SIZE + FOOTER_SIZE {
        return Err(0);
    }
    data[0] = b'b';
    data[1] = b'r';
    data[2] = id;
    data[3] = 0;
    Ok(HEADER_SIZE)
}

pub fn append_can_frame(data: &mut [u8], index: usize, msg: &CanMsg) -> Result<usize, usize> {
    let payload_len = msg.length as usize;
    if data.len() - index < payload_len + CAN_FRAME_OVERHEAD + FOOTER_SIZE {
        return Err(index);
    }
    data[index] = b'c';
    data[index + 1..index + 3].copy_from_slice(&msg.id.to_le_bytes());
    data[index + 3] = msg.length as u8;
    data[index + 4..index + 4 + payload_len].copy_from_slice(&msg.data[0..payload_len]);
    Ok(index + CAN_FRAME_OVERHEAD + payload_len)
}

pub fn finish_buffer<'a>(data: &'a mut [u8], index: usize) -> Result<&'a [u8], ()> {
    if data.len() - index < FOOTER_SIZE {
        return Err(());
    }
    if index < HEADER_SIZE {
        return Err(());
    }
    let payload_len = index - HEADER_SIZE;
    if payload_len > u8::MAX as usize {
        return Err(());
    }
    data[3] = payload_len as u8;
    let checksum: u16 = X25.checksum(&data[..index]);
    let footer_start = index;
    let footer_end = footer_start + FOOTER_SIZE;
    data[footer_start..footer_end].copy_from_slice(&checksum.to_le_bytes());
    Ok(&data[..footer_end])
}

pub enum MsgError {
    InvalidHeader,
    InvalidCrc,
    InvalidLen,
}

pub fn verify_and_extract_messages(data: &[u8]) -> Result<&[u8], MsgError> {
    if data.len() < HEADER_SIZE + FOOTER_SIZE {
        return Err(MsgError::InvalidLen);
    }
    if &data[..2] != b"br" {
        return Err(MsgError::InvalidHeader);
    };
    let len = data[3] as usize;
    if data.len() != len + HEADER_SIZE + FOOTER_SIZE {
        return Err(MsgError::InvalidLen);
    };
    let payload_end = HEADER_SIZE + len;
    let calculated_checksum: u16 = X25.checksum(&data[..payload_end]);
    let checksum_start = payload_end;
    let checksum_end = checksum_start + FOOTER_SIZE;
    let received_checksum: u16 =
        u16::from_le_bytes(data[checksum_start..checksum_end].try_into().unwrap());
    if calculated_checksum != received_checksum {
        return Err(MsgError::InvalidCrc);
    };

    Ok(&data[HEADER_SIZE..payload_end])
}

pub fn consume_msg(data: &[u8]) -> Result<(Msg, &[u8]), MsgError> {
    if data.is_empty() {
        return Err(MsgError::InvalidLen);
    }
    let ret;
    let index: usize;
    match data[0] {
        b'c' => {
            if data.len() < CAN_FRAME_OVERHEAD {
                return Err(MsgError::InvalidLen);
            }
            let id = u16::from_le_bytes(data[1..3].try_into().unwrap());
            let length = data[3];
            if data.len() < CAN_FRAME_OVERHEAD + length as usize {
                return Err(MsgError::InvalidLen);
            }
            let mut can_msg = CanMsg {
                id: id,
                length: length,
                data: [0; 64],
            };
            can_msg.data[..length as usize].copy_from_slice(&data[4..4 + length as usize]);
            index = 4 + length as usize;
            ret = Msg::Can(can_msg);
        }

        _ => return Err(MsgError::InvalidHeader),
    };

    Ok((ret, &data[index..]))
}

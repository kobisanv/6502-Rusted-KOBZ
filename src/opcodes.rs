// Import necessary modules and dependencies
use crate::CPU::AddressingMode;
use std::collections::HashMap;
use lazy_static::lazy_static;

// Define a struct representing an OpCode
pub struct OpCode {
    pub c: u8,                   // OpCode value
    pub mnemonic: &'static str,  // Mnemonic representing the OpCode
    pub l: u8,                   // Length of the OpCode in bytes
    pub cyc: u8,                 // Number of cycles required for the OpCode
    pub mode: AddressingMode,    // Addressing mode used by the OpCode
}

// Implementation block for the OpCode struct
impl OpCode {
    // Constructor for creating a new OpCode instance
    fn new(c: u8, mnemonic: &'static str, l: u8, cyc: u8, mode: AddressingMode) -> Self {
        OpCode {
            c,
            mnemonic,
            l,
            cyc,
            mode,
        }
    }
}

// Lazy-static initialization of CPU_OP_CODES and OPCODES_MAP
lazy_static! {
    // Vector containing instances of OpCode representing CPU opcodes
    pub static ref CPU_OP_CODES: Vec<OpCode> = vec![
        OpCode::new(0x00, "BRK", 1, 7, AddressingMode::NoneAddressing),
        OpCode::new(0xaa, "TAX", 1, 2, AddressingMode::NoneAddressing),
        OpCode::new(0xe8, "INX", 1, 2, AddressingMode::NoneAddressing),

        OpCode::new(0xa9, "LDA", 2, 2, AddressingMode::I),
        OpCode::new(0xa5, "LDA", 2, 3, AddressingMode::ZP),
        OpCode::new(0xb5, "LDA", 2, 4, AddressingMode::ZP_X),
        OpCode::new(0xad, "LDA", 3, 4, AddressingMode::ABS),
        OpCode::new(0xbd, "LDA", 3, 4, AddressingMode::ABS_X),
        OpCode::new(0xb9, "LDA", 3, 4, AddressingMode::ABS_Y),
        OpCode::new(0xa1, "LDA", 2, 6, AddressingMode::IND_X),
        OpCode::new(0xb1, "LDA", 2, 5, AddressingMode::IND_Y),
    ];

    // HashMap mapping OpCode values to references of OpCode instances
    pub static ref OPCODES_MAP: HashMap<u8, &'static OpCode> = {
        let mut map = HashMap::new();
        // Populate the HashMap with OpCode values as keys and references to OpCode instances as values
        for cpuop in &*CPU_OP_CODES {
            map.insert(cpuop.c, cpuop);
        }
        map
    };
}

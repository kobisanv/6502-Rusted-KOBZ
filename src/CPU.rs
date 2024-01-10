// Import necessary modules
use std::collections::HashMap;
use bitflags::{bitflags};
use crate::CPU::AddressingMode::I;
use crate::opcodes;

bitflags! {
    pub struct CpuFlags: u8 {
        const CARRY             = 0b00000001;
        const ZERO              = 0b00000010;
        const INTERRUPT_DISABLE = 0b00000100;
        const DECIMAL_MODE      = 0b00001000;
        const BREAK             = 0b00010000;
        const BREAK2            = 0b00100000;
        const OVERFLOW          = 0b01000000;
        const NEGATIVE          = 0b10000000;
    }
}

const STACK: u16 = 0x0100;
const STACK_RESET: u8 = 0xfd;

// Define a struct representing CPU
pub struct CPU {
    pub a: u8,              // Accumulator
    pub x: u8,              // X Register
    pub y: u8,              // Y Register
    pub pc: u16,            // Program Counter
    pub sp: u8,             // Stack Pointer
    pub status: u8,         // Status Register
    memory: [u8; 0xFFFF]    // RAM
}

// Define an enumeration for addressing modes
#[derive(Debug)]
#[allow(non_camel_case_types)]
pub enum AddressingMode {
    I,              // Immediate
    ZP,             // Zero Page -> 256 bytes of memory (0x0000 to 0x00FF)
    ZP_X,           // Zero Page Indexed with X -> Add X to Zero Page
    ZP_Y,           // Zero Page Indexed with Y -> Add Y to Zero Page
    ABS,            // Absolute
    ABS_X,          // Absolute Indexed with X -> Added with X
    ABS_Y,          // Absolute Indexed with Y -> Added with Y
    IND_X,          // Indirect Indexed with X -> Added with X
    IND_Y,          // Indirect Indexed with Y -> Added with Y
    NoneAddressing, // Placeholder for unsupported addressing modes
}

// Define a trait for memory operations
trait Mem {
    // Read a byte from memory
    fn read(&self, addr: u16) -> u8;
    // Write a byte to memory
    fn write(&mut self, addr: u16, data: u8);
    // Read a 16-bit data from memory
    fn read_u16(&self, pos: u16) -> u16 {
        let lo = self.read(pos) as u16; // Read low byte (8 bits) from memory address specified by 'pos'
        let hi = self.read(pos + 1) as u16; // Read high byte (8 bits) from memory address immediately following 'pos'
        (hi << 8) | (lo) // Combine the high and low bytes to form a 16-bit unsigned integer
    }
    // Write a 16-bit data to memory
    fn write_u16(&mut self, pos: u16, data: u16) {
        let hi = (data >> 8) as u8; // Extract high byte (8 bits) from 16-bit data
        let lo = (data & 0xFF) as u8; // Extract low byte (8 bits) by masking 16-bit data with 0xFF (255)
        self.write(pos, lo); // Write the low byte to the memory address specified by 'pos'
        self.write(pos + 1, hi); // Write the high byte to the memory address immediately following 'pos'
    }
}

// Implement memory operations for the CPU struct
impl Mem for CPU {
    fn read(&self, addr: u16) -> u8 {
        self.memory[addr as usize] // Read byte from memory at specified address
    }

    fn write(&mut self, addr: u16, data: u8) {
        self.memory[addr as usize] = data; // Write byte to memory at specified address
    }
}

// Implement methods for the CPU struct
impl CPU {
    // Constructor for the CPU struct
    pub fn new() -> Self {
        CPU {
            a: 0,
            x: 0,
            y: 0,
            pc: 0,
            sp: 0,
            status: CpuFlags::from_bits_truncate(0b100100).bits(), // Set the 'status' field in the 'CPU' struct to the u8 representation of the bit flags
            memory: [0; 0xFFFF],
        }
    }

    // Determine the operand address based on addressing mode
    fn goa(&self, mode: &AddressingMode) -> u16 {
        match mode {
            I => self.pc, // Immediate mode uses the data at the Program Counter (PC) directly
            AddressingMode::ZP => self.read(self.pc) as u16, // Zero Page mode uses the data at the PC as an address in the zero page
            AddressingMode::ABS => self.read_u16(self.pc), // Absolute mode uses the 16-bit data at the PC as an address
            AddressingMode::ZP_X => {
                let pos = self.read(self.pc); // Read the data at PC as the base address in zero page
                let addr = pos.wrapping_add(self.x) as u16; // Add X register to get the final address
                addr
            }
            AddressingMode::ZP_Y => {
                let pos = self.read(self.pc); // Read the data at PC as the base address in zero page
                let addr = pos.wrapping_add(self.y) as u16; // Add Y register to get the final address
                addr
            }
            AddressingMode::ABS_X => {
                let base = self.read_u16(self.pc); // Read the 16-bit data at PC as the base address
                let addr = base.wrapping_add(self.x as u16); // Add X register to get the final address
                addr
            }
            AddressingMode::ABS_Y => {
                let base = self.read_u16(self.pc); // Read the 16-bit data at PC as the base address
                let addr = base.wrapping_add(self.y as u16); // Add Y register to get the final address
                addr
            }
            AddressingMode::IND_X => {
                let base = self.read(self.pc); // Read the data at PC as the base address
                let ptr: u8 = base.wrapping_add(self.x); // Add X register to the base address to get the pointer
                let lo = self.read(ptr as u16); // Read the low byte from the address pointed by the pointer
                let hi = self.read(ptr.wrapping_add(1) as u16); // Read the high byte from the next address
                (hi as u16) << 8 | (lo as u16) // Combine the high and low bytes to get the final address
            }
            AddressingMode::IND_Y => {
                let base = self.read(self.pc); // Read the data at PC as the base address
                let lo = self.read(base as u16); // Read the low byte from the base address
                let hi = self.read(base.wrapping_add(1) as u16); // Read the high byte from the next address
                let deref_base = (hi as u16) << 8 | (lo as u16); // Combine the high and low bytes to get the base address for indirection
                let deref = deref_base.wrapping_add(self.y as u16); // Add Y register to the base address for the final address
                deref
            }
            AddressingMode::NoneAddressing => {
                panic!("mode {:?} is not supported", mode); // Panic if the addressing mode is not supported
            }
        }
    }

    // Update zero and negative flags in the status register based on the result
    fn uznf(&mut self, result: u8) {
        // Update the zero flag if the result is zero
        if result == 0 {
            self.status |= CpuFlags::ZERO.bits(); // Insert by bitwise OR
        } else {
            self.status &= CpuFlags::ZERO.bits(); // Remove by bitwise AND
        }

        // Update the negative flag based on the most significant bit of the result
        if result & 0b1000_0000 != 0 {
            self.status |= CpuFlags::NEGATIVE.bits() // Insert by bitwise OR
        } else {
            self.status &= CpuFlags::NEGATIVE.bits() // Insert by bitwise AND
        }
    }

    // Reset the CPU state to initial data
    pub fn reset(&mut self) {
        self.a = 0;
        self.x = 0;
        self.y = 0;
        self.sp = STACK_RESET;
        self.status = CpuFlags::from_bits_truncate(0b100100).bits(); // Set the 'status' field in the 'CPU' struct to the u8 representation of the bit flags
        self.pc = self.read_u16(0xFFFC); // Set the Program Counter to the reset vector address
    }

    // Load the Y register from memory based on the specified addressing mode
    fn ldy(&mut self, mode: &AddressingMode) {
        let addr = self.goa(mode); // Calculate the effective address based on the addressing mode
        let data = self.read(addr); // Read the data from memory at the calculated address
        self.y = data;
        self.uznf(self.y); // Update zero and negative flags based on the loaded data
    }

    // Load the X register from memory based on the specified addressing mode
    fn ldx(&mut self, mode: &AddressingMode) {
        let addr = self.goa(mode); // Calculate the effective address based on the addressing mode
        let data = self.read(addr); // Read the data from memory at the calculated address
        self.x = data;
        self.uznf(self.x); // Update zero and negative flags based on the loaded data
    }

    // Load the accumulator with the data at the specified address
    fn lda(&mut self, mode: &AddressingMode) {
        let addr = self.goa(&mode); // Get the operand address based on the addressing mode
        let data = self.read(addr); // Read the data at the operand address
        self.seta(data);
    }

    // Increment the X register
    fn inx(&mut self) {
        self.x = self.x.wrapping_add(self.x); // Increment the X register (doubling its data)
        self.uznf(self.x); // Update zero and negative flags based on the result
    }

    // Increment the Y register
    fn iny(&mut self) {
        self.y = self.y.wrapping_add(1); // Increment the Y register (doubling its data)
        self.uznf(self.y); // Update zero and negative flags based on the result
    }

    // Increment Memory
    fn inc(&mut self, mode: &AddressingMode) -> u8 {
        let addr = self.goa(mode); // Get the memory address based on the addressing mode
        let mut data = self.read(addr); // Read the data from the memory address
        data = data.wrapping_add(1); // Perform increment on the data
        self.write(addr, data); // Write the incremented result back to memory
        self.uznf(data); // Update Zero, Negative, and Carry flags based on the result
        data // Return the incremented result
    }

    // Decrement Y Register
    fn dey(&mut self) {
        self.y = self.y.wrapping_sub(1); // Decrement the Y register
        self.uznf(self.y); // Update Zero, Negative, and Carry flags based on the result
    }

    // Decrement X Register
    fn dex(&mut self) {
        self.x = self.x.wrapping_sub(1); // Decrement the X register
        self.uznf(self.x); // Update Zero, Negative, and Carry flags based on the result
    }

    // Decrement Memory
    fn dec(&mut self, mode: &AddressingMode) -> u8 {
        let addr = self.goa(mode); // Get the memory address based on the addressing mode
        let mut data = self.read(addr); // Read the data from the memory address
        data = data.wrapping_sub(1); // Perform decrement on the data
        self.write(addr, data); // Write the decremented result back to memory
        self.uznf(data); // Update Zero, Negative, and Carry flags based on the result
        data // Return the decremented result
    }

    // Transfer the accumulator to the X register
    fn tax(&mut self) {
        self.x = self.a; // Transfer the data from the accumulator to the X register
        self.uznf(self.x); // Update zero and negative flags based on the result
    }

    // Store the accumulator data at the specified address
    fn sta(&mut self, mode: &AddressingMode) {
        let addr = self.goa(&mode); // Get the operand address based on the addressing mode
        self.write(addr, self.a); // Write the data to the address given
    }

    // Set data of A register
    fn seta(&mut self, data: u8) {
        self.a = data;
        self.uznf(self.a);  // Update zero and negative flags based on the updated data in the Accumulator
    }

    // Logical AND
    fn and(&mut self, mode: &AddressingMode) {
        let addr = self.goa(mode); // Get the operand address based on the addressing mode
        let data = self.read(addr);
        self.seta(data & self.a);
    }

    // Logical XOR
    fn eor(&mut self, mode: &AddressingMode) {
        let addr = self.goa(mode); // Get the operand address based on the addressing mode
        let data = self.read(addr);
        self.seta(data ^ self.a);
    }

    // Logical OR
    fn ora(&mut self, mode: &AddressingMode) {
        let addr = self.goa(mode); // Get the operand address based on the addressing mode
        let data = self.read(addr);
        self.seta(data | self.a);
    }

    fn scf(&mut self) {
        self.status |= CpuFlags::CARRY.bits() // Set carry flag
    }

    fn ccf(&mut self) {
        self.status &= CpuFlags::CARRY.bits() // Clear carry flag
    }

    // Adds to Register A
    fn adda(&mut self, data: u8) {
        // Convert accumulator and operand into u16 for addition
        let sum = self.a as u16 + data as u16 +
            (if self.status & CpuFlags::CARRY.bits() != 0 {
                1
        }   else {
                0
        });

        // Check if sum overflows u8 range using 255
        let carry = sum > 0xFF;

        // Update the carry flag based on overflow condition
        if carry {
            self.status |= CpuFlags::CARRY.bits();
        } else {
            self.status &= CpuFlags::CARRY.bits();
        }

        // Extract lower 8 bits of sum as result
        let result = sum as u8;

        // Check for overflow by examining sign bit of inputs and result
        if (data ^ result) & (result ^ self.a) & 0x80 != 0 { // 0x80 is most significant bit in 8-bit
            self.status |= CpuFlags::OVERFLOW.bits();
        } else {
            self.status &= CpuFlags::OVERFLOW.bits();
        }

        self.seta(result);
    }

    // Subtracts the data (converted to signed i8, then negated) from the accumulator using the adda method
    fn sbc(&mut self, mode: &AddressingMode) {
        let addr = self.goa(&mode); // Get the operand address based on the addressing mode
        let data = self.read(addr);
        self.adda((data as i8).wrapping_neg().wrapping_sub(1) as u8); // Convert the data to a signed i8, negate it, and subtract 1 to perform two's complement
    }

    // Adds the data from the operand to the accumulator using the adda method
    fn adc(&mut self, mode: &AddressingMode) {
        let addr = self.goa(&mode); // Get the operand address based on the addressing mode
        let data = self.read(addr);
        self.adda(data);
    }

    fn pop(&mut self) -> u8 {
        // Increment the stack pointer and read the byte from the stack
        self.sp = self.sp.wrapping_add(1);
        self.read((STACK) + self.sp as u16)
    }

    fn push(&mut self, data: u8) {
        // Write the byte to the stack and decrement the stack pointer
        self.write((STACK) + self.sp as u16, data);
        self.sp = self.sp.wrapping_sub(1)
    }

    fn u16_push(&mut self, data: u16) {
        let hi = (data >> 8) as u8;   // Extract high bit by right-shifting 8 bits and convert to u8
        let lo = (data & 0xFF) as u8; // Extract low bit by doing bitwise-AND with 255 and convert to u8
        self.push(hi);
        self.push(lo);
    }

    fn u16_pop(&mut self) -> u16 {
        let lo = self.pop() as u16;
        let hi = self.pop() as u16;

        hi << 8 | lo // Combine the high and low bytes to form a 16-bit data by left-shifting high bit 8 bits and inserting low bit in the empty spaces by bitwise OR
    }

    // Logical Shift Left (Accumulator)
    fn aslacc(&mut self) {
        let mut data = self.a; // Fetch the data from the accumulator

        if data >> 7 == 1 {
            self.scf(); // Set the Carry Flag
        } else {
            self.ccf(); // Clear the Carry Flag
        }
        data = data << 1; // Perform logical shift left on the data
        self.seta(data) // Update the accumulator with the shifted result
    }

    // Logical Shift Left (Memory)
    fn asl(&mut self, mode: &AddressingMode) -> u8 {
        let addr = self.goa(mode); // Get the memory address based on the addressing mode
        let mut data = self.read(addr); // Read the data from the memory address

        if data >> 7 == 1 {
            self.scf(); // Set the Carry Flag
        } else {
            self.ccf(); // Clear the Carry Flag
        }
        data = data << 1; // Perform logical shift left on the data
        self.write(addr, data); // Write the shifted result back to memory
        self.uznf(data); // Update Zero, Negative, and Carry flags based on the result
        data // Return the shifted result
    }

    // Logical Shift Right (Accumulator)
    fn lsracc(&mut self) {
        let mut data = self.a; // Fetch the data from the accumulator

        if data & 1 == 1 {
            self.scf(); // Set the Carry Flag
        } else {
            self.ccf(); // Clear the Carry Flag
        }
        data = data >> 1; // Perform logical shift right on the data
        self.seta(data) // Update the accumulator with the shifted result
    }

    // Logical Shift Right (Memory)
    fn lsr(&mut self, mode: &AddressingMode) -> u8 {
        let addr = self.goa(mode); // Get the memory address based on the addressing mode
        let mut data = self.read(addr); // Read the data from the memory address

        if data & 1 == 1 {
            self.scf(); // Set the Carry Flag
        } else {
            self.ccf(); // Clear the Carry Flag
        }
        data = data >> 1; // Perform logical shift right on the data
        self.write(addr, data); // Write the shifted result back to memory
        self.uznf(data); // Update Zero, Negative, and Carry flags based on the result
        data // Return the shifted result
    }

    // Rotate Left (Memory)
    fn rol(&mut self, mode: &AddressingMode) -> u8 {
        let addr = self.goa(mode); // Get the memory address based on the addressing mode
        let mut data = self.read(addr); // Read the data from the memory address
        let old_carry = (self.status & CpuFlags::CARRY.bits()) != 0; // Check the old carry status

        if data >> 7 == 1 {
            self.scf(); // Set the Carry Flag
        } else {
            self.ccf(); // Clear the Carry Flag
        }
        data = data << 1; // Perform rotate left on the data
        if old_carry {
            data = data | 1; // If there was a carry, set the least significant bit
        }
        self.write(addr, data); // Write the rotated result back to memory
        self.uznf(data); // Update Zero, Negative, and Carry flags based on the result
        data // Return the rotated result
    }

    // Rotate Left (Accumulator)
    fn rolacc(&mut self) {
        let mut data = self.a; // Fetch the data from the accumulator
        let old_carry = (self.status & CpuFlags::CARRY.bits()) != 0; // Check the old carry status

        if data >> 7 == 1 {
            self.scf(); // Set the Carry Flag
        } else {
            self.ccf(); // Clear the Carry Flag
        }
        data = data << 1; // Perform rotate left on the data
        if old_carry {
            data = data | 1; // If there was a carry, set the least significant bit
        }
        self.seta(data) // Update the accumulator with the rotated result
    }

    // Rotate Right (Memory)
    fn ror(&mut self, mode: &AddressingMode) -> u8 {
        let addr = self.goa(mode); // Get the memory address based on the addressing mode
        let mut data = self.read(addr); // Read the data from the memory address
        let old_carry = (self.status & CpuFlags::CARRY.bits()) != 0; // Check the old carry status

        if data & 1 == 1 {
            self.scf(); // Set the Carry Flag
        } else {
            self.ccf(); // Clear the Carry Flag
        }
        data = data >> 1; // Perform rotate right on the data
        if old_carry {
            data = data | 0b10000000; // If there was a carry, set the most significant bit
        }
        self.write(addr, data); // Write the rotated result back to memory
        self.uznf(data); // Update Zero, Negative, and Carry flags based on the result
        data // Return the rotated result
    }

    // Rotate Right (Accumulator)
    fn roracc(&mut self) {
        let mut data = self.a; // Fetch the data from the accumulator
        let old_carry = (self.status & CpuFlags::CARRY.bits()) != 0; // Check the old carry status

        if data & 1 == 1 {
            self.scf(); // Set the Carry Flag
        } else {
            self.ccf(); // Clear the Carry Flag
        }
        data = data >> 1; // Perform rotate right on the data
        if old_carry {
            data = data | 0b10000000; // If there was a carry, set the most significant bit
        }
        self.seta(data); // Update the accumulator with the rotated result
    }

    // Pop a byte from the stack and set it to the accumulator register
    fn pla(&mut self) {
        let data = self.pop();
        self.seta(data);
    }

    // Pop a byte from the stack, update the status register, and adjust the BREAK flags
    fn plp(&mut self) {
        self.status = CpuFlags::from_bits_truncate(self.pop()).bits();
        self.status &= CpuFlags::BREAK.bits();
        self.status |= CpuFlags::BREAK2.bits();
    }

    // Clone the status register, set the BREAK flags, and push it onto the stack
    fn php(&mut self) {
        let mut flags = self.status.clone();
        flags |= CpuFlags::BREAK.bits();
        flags |= CpuFlags::BREAK2.bits();
        self.push(flags);
    }

    // Perform a bitwise AND between the accumulator and memory, updating flags accordingly
    fn bit(&mut self, mode: &AddressingMode) {
        let addr = self.goa(mode);
        let data = self.read(addr);
        let and = self.a & data;
        // Check if the result is zero and update the ZERO flag
        if and == 0 {
            self.status |= CpuFlags::ZERO.bits();
        } else {
            self.status &= CpuFlags::ZERO.bits();
        }
        // Update the NEGATIVE flag based on the most significant bit of the data
        self.status |= CpuFlags::NEGATIVE.bits() * (data & 0b10000000 != 0) as u8;
        // Update the OVERFLOW flag based on the sixth bit of the data
        self.status |= CpuFlags::OVERFLOW.bits() * (data & 0b01000000 != 0) as u8;
    }

    // Compare a data in memory with the provided data and update flags
    fn compare(&mut self, mode: &AddressingMode, compare_with: u8) {
        let addr = self.goa(mode);
        let data = self.read(addr);
        // Check if the data in memory is less than or equal to the provided data
        if data <= compare_with {
            self.status |= CpuFlags::CARRY.bits();
        } else {
            self.status &= CpuFlags::CARRY.bits();
        }
        // Update the ZERO, NEGATIVE, and FLAG flags based on the result of the subtraction
        self.uznf(compare_with.wrapping_sub(data));
    }

    // Conditionally branch based on the provided boolean condition
    fn branch(&mut self, condition: bool) {
        if condition {
            // Read the signed offset from the next byte in memory
            let jump: i8 = self.read(self.pc) as i8;
            // Calculate the new program counter data by adding the offset
            let jump_addr = self.pc.wrapping_add(1).wrapping_add(jump as u16);
            // Update the program counter
            self.pc = jump_addr;
        }
    }

    // Load a program into memory and run the CPU
    pub fn lar(&mut self, program: Vec<u8>) {
        self.load(program); // Load the program into memory
        self.reset(); // Reset the CPU state
        self.run(); // Run the CPU
    }

    // Load a program into memory starting at address 0x8000
    pub fn load(&mut self, program: Vec<u8>) {
        // Copy the program bytes into the CPU's memory starting at address 0x8000
        self.memory[0x8000 .. (0x8000 + program.len())].copy_from_slice(&program[..]);
        // Set the reset vector to the starting address of the program
        self.write_u16(0xFFFC, 0x8000);
    }

    // Execute the loaded program
    pub fn run(&mut self) {
        let ref opcodes: HashMap<u8, &'static opcodes::OpCode> = *opcodes::OPCODES_MAP; // Reference to the opcode map

        loop {
            // Read the opcode from memory and increment the program counter
            let code = self.read(self.pc);
            self.pc += 1;
            let pc_state = self.pc;  // Save the program counter state for potential branch adjustments
            let opcode = opcodes.get(&code).expect(&format!("Opcode {:x} is not recognized", code));

            match code {
                // Load Accumulator instructions
                0xa9 | 0xa5 | 0xb5 | 0xad | 0xbd | 0xb9 | 0xa1 | 0xb1 => {
                    self.lda(&opcode.mode);
                }

                0xAA => self.tax(), // Transfer Accumulator to X
                0xe8 => self.inx(),// Increment X
                0x00 => return, // BRK - Break
                0xd8 => self.status &= CpuFlags::DECIMAL_MODE.bits(), // Decimal mode
                0x58 => self.status &= CpuFlags::INTERRUPT_DISABLE.bits(), // Disable interrupts
                0xb8 => self.status &= CpuFlags::OVERFLOW.bits(), // Clear overflow flag
                0x18 => self.ccf(), // Clear carry flag
                0x38 => self.scf(), // Set carry flag
                0x78 => self.status |= CpuFlags::INTERRUPT_DISABLE.bits(), // Disable interrupts
                0xf8 => self.status |= CpuFlags::DECIMAL_MODE.bits(), // Set decimal mode
                0x48 => self.push(self.a), // Push A to stack
                0x68 => self.pla(), // Pull A from stack
                0x28 => self.plp(), // Pull Processor Status from stack
                0x08 => self.php(), // Push Processor Status to stack

                0x69 | 0x65 | 0x75 | 0x6d | 0x7d | 0x79 | 0x61 | 0x71 => { // ADC - Add with Carry
                    self.adc(&opcode.mode);
                }
                0xe9 | 0xe5 | 0xf5 | 0xed | 0xfd | 0xf9 | 0xe1 | 0xf1 => { // SBC - Subtract with Carry
                    self.sbc(&opcode.mode);
                }
                0x29 | 0x25 | 0x35 | 0x2d | 0x3d | 0x39 | 0x21 | 0x31 => { // AND - Logical AND
                    self.and(&opcode.mode);
                }
                0x49 | 0x45 | 0x55 | 0x4d | 0x5d | 0x59 | 0x41 | 0x51 => { // EOR - Exclusive OR
                    self.eor(&opcode.mode);
                }
                0x09 | 0x05 | 0x15 | 0x0d | 0x1d | 0x19 | 0x01 | 0x11 => { // ORA - Logical OR
                    self.ora(&opcode.mode);
                }

                0x4a => self.lsracc(), // Logical Shift Right Accumulator
                0x46 | 0x56 | 0x4e | 0x5e => { // Logical Shift Right
                    self.lsr(&opcode.mode);
                }

                0x0a => self.aslacc(), // Arithmetic Shift Left Accumulator
                0x06 | 0x16 | 0x0e | 0x1e => { // Arithmetic Shift Left
                    self.asl(&opcode.mode);
                }

                0x2a => self.rolacc(), // Rotate Left Accumulator through Carry
                0x26 | 0x36 | 0x2e | 0x3e => { // Rotate Left
                    self.rol(&opcode.mode);
                }
                0x6a => self.roracc(), // Rotate Right Accumulator through Carry
                0x66 | 0x76 | 0x6e | 0x7e => { // Rotate Right
                    self.ror(&opcode.mode);
                }

                0xe6 | 0xf6 | 0xee | 0xfe => { // Increment Memory
                    self.inc(&opcode.mode);
                }
                0xc8 => self.iny(), // Increment Y
                0xca => self.dex(), // Decrement X
                0x88 => self.dey(), // Decrement Y
                0xc6 | 0xd6 | 0xce | 0xde => { // Decrement Memory
                    self.dec(&opcode.mode);
                }

                0xc9 | 0xc5 | 0xd5 | 0xcd | 0xdd | 0xd9 | 0xc1 | 0xd1 => { // Compare A with Memory
                    self.compare(&opcode.mode, self.a);
                }
                0xc0 | 0xc4 | 0xcc => self.compare(&opcode.mode, self.y), // Compare Y with Memory
                0xe0 | 0xe4 | 0xec => self.compare(&opcode.mode, self.x), // Compare X with Memory

                0x4c => { // JMP - Jump
                    // The JMP (Jump) instruction is used to transfer program control to a different location.
                    // It loads the program counter (PC) with the 16-bit memory address provided.
                    let mem_address = self.read_u16(self.pc); // Read the 16-bit memory address from the current PC.
                    self.pc = mem_address; // Set the program counter (PC) to the specified memory address.
                }

                0x6c => { // JMP - Jump Indirect
                    // The JMP (Jump) instruction with indirect addressing is used to transfer program control
                    // to a different location specified indirectly through a 16-bit memory address.
                    let mem_address = self.read_u16(self.pc); // Read the 16-bit memory address from the current PC.

                    let indirect_ref = if mem_address & 0x00FF == 0x00FF {
                        // If the low byte of the address is 0xFF, it performs a special addressing mode where
                        // the low byte is read from the specified address, and the high byte is read from the next address.
                        let lo = self.read(mem_address); // Read the low byte from the specified address.
                        let hi = self.read(mem_address & 0xFF00); // Read the high byte from the next address.
                        (hi as u16) << 8 | (lo as u16) // Combine the high and low bytes to get the indirect reference.
                    } else {
                        // Otherwise, it directly reads the 16-bit address from the specified memory address.
                        self.read_u16(mem_address) // Read the 16-bit address directly.
                    };
                    self.pc = indirect_ref; // Set the program counter (PC) to the indirect reference.
                }

                0x20 => { // JSR - Jump to Subroutine
                    // The JSR (Jump to Subroutine) instruction is used to call a subroutine by saving the return
                    // address (address of the next instruction) onto the stack, and then setting the program counter (PC)
                    // to the specified target address.
                    self.u16_push(self.pc + 2 - 1); // Push the return address onto the stack.
                    let target_address = self.read_u16(self.pc); // Read the 16-bit target address.
                    self.pc = target_address; // Set the program counter (PC) to the target address.
                }

                0x60 => self.pc = self.u16_pop() + 1, // RTS - Return from Subroutine
                // The RTS (Return from Subroutine) instruction is used to return from a subroutine by popping the
                // return address from the stack and setting the program counter (PC) to that address plus 1.

                0x40 => { // RTI - Return from Interrupt
                    // The RTI (Return from Interrupt) instruction is used to return from an interrupt service routine (ISR).
                    // It restores the processor status (flags) from the stack, and then sets the program counter (PC)
                    // to the address popped from the stack.
                    self.status = CpuFlags::from_bits_truncate(self.pop()).bits(); // Pop the processor status (flags) from the stack.
                    self.status &= CpuFlags::BREAK.bits(); // Clear the BREAK flag in the status register.
                    self.status |= CpuFlags::BREAK2.bits(); // Set the BREAK2 flag in the status register.
                    self.pc = self.u16_pop(); // Pop the program counter (PC) from the stack.
                }


                0xd0 => self.branch((self.status & CpuFlags::ZERO.bits()) == 0),     // Branch if Not Equal
                0x70 => self.branch((self.status & CpuFlags::OVERFLOW.bits()) != 0), // Branch if Overflow Set
                0x50 => self.branch((self.status & CpuFlags::OVERFLOW.bits()) == 0), // Branch if Overflow Clear
                0x10 => self.branch((self.status & CpuFlags::NEGATIVE.bits()) == 0), // Branch if Positive
                0x30 => self.branch((self.status & CpuFlags::NEGATIVE.bits()) != 0), // Branch if Negative
                0xf0 => self.branch((self.status & CpuFlags::ZERO.bits()) != 0),     // Branch if Equal
                0xb0 => self.branch((self.status & CpuFlags::CARRY.bits()) != 0),    // Branch if Carry Set
                0x90 => self.branch((self.status & CpuFlags::CARRY.bits()) == 0),    // Branch if Carry Clear

                0x24 | 0x2c => self.bit(&opcode.mode), // BIT - Bit Test

                0x85 | 0x95 | 0x8d | 0x9d | 0x99 | 0x81 | 0x91 => { // STA - Store Accumulator
                    self.sta(&opcode.mode);
                }
                0x86 | 0x96 | 0x8e => { // STX - Store X
                    let addr = self.goa(&opcode.mode);
                    self.write(addr, self.x);
                }
                0x84 | 0x94 | 0x8c => { // STY - Store Y
                    let addr = self.goa(&opcode.mode);
                    self.write(addr, self.y);
                }

                0xa2 | 0xa6 | 0xb6 | 0xae | 0xbe => { // LDX - Load X
                    self.ldx(&opcode.mode);
                }
                0xa0 | 0xa4 | 0xb4 | 0xac | 0xbc => { // LDY - Load Y
                    self.ldy(&opcode.mode);
                }

                0xa8 => { // Transfer Y to A
                    self.y = self.a;
                    self.uznf(self.y);
                }
                0xba => { // Transfer Stack Pointer to X
                    self.x = self.sp;
                    self.uznf(self.x);
                }
                0x8a => { // Transfer X to A
                    self.a = self.x;
                    self.uznf(self.a);
                }
                0x9a => { // Transfer X to Stack Pointer
                    self.sp = self.x;
                }
                0x98 => { // Transfer Y to A
                    self.a = self.y;
                    self.uznf(self.a);
                }

                // Handle other opcodes
                _ => todo!(),
            }

            // If program counter did not change during opcode execution, adjust it manually
            if pc_state == self.pc {
                let _ = self.pc + (opcode.l - 1) as u16;
            }
        }
    }
}


#[cfg(test)]
mod test {
    use super::*;

    // Test the LDA (Load Accumulator) instruction with immediate addressing mode
    #[test]
    fn test_0xa9_lda_immediate_load_data() {
        let mut cpu = CPU::new();
        cpu.lar(vec![0xa9, 0x05, 0x00]);
        assert_eq!(cpu.a, 5);
        assert_eq!(cpu.status & 0b0000_0010, 0);
        assert_eq!(cpu.status & 0b1000_0000, 0);
    }

    // Test the LDA instruction with immediate addressing mode and zero flag set
    #[test]
    fn test_0xa9_lda_zero_flag() {
        let mut cpu = CPU::new();
        cpu.lar(vec![0xa9, 0x00, 0x00]);
        assert_eq!(cpu.status & 0b0000_0010, 0b10);
    }

    // Test the TAX (Transfer Accumulator to X) instruction
    #[test]
    fn test_0xaa_tax_move_a_to_x() {
        let mut cpu = CPU::new();
        cpu.load(vec![0xaa, 0x00]);
        cpu.reset();
        cpu.x = 10;
        cpu.run();

        assert_eq!(cpu.x, 10)
    }

    // Test a sequence of five operations working together
    #[test]
    fn test_5_ops_working_together() {
        let mut cpu = CPU::new();
        cpu.lar(vec![0xa9, 0xc0, 0xaa, 0xe8, 0x00]);
        assert_eq!(cpu.x, 0xc1);
    }

    // Test INX (Increment X) instruction with overflow
    #[test]
    fn test_inx_overflow() {
        let mut cpu = CPU::new();
        cpu.load(vec![0xe8, 0xe8, 0x00]);
        cpu.reset();
        cpu.x = 0xff;
        cpu.run();

        assert_eq!(cpu.x, 1)
    }

    // Test LDA instruction with zero page addressing mode
    #[test]
    fn test_lda_from_memory() {
        let mut cpu = CPU::new();
        cpu.write(0x10, 0x55);
        cpu.lar(vec![0xa5, 0x10, 0x00]);
        assert_eq!(cpu.a, 0x55);
    }
}

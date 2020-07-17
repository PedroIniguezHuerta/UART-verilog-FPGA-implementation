//============================================================================
// PROGRAM NAME:
//              RS232TxRx
// DESCRIPTION:
//              The following is the RS232 module to communicate with FPGA leds and Computer HyperTerminal:
//              9600 bds, 8 data bits, odd parity, no Hardware flow
//
// AUTHOR:
//              Pedro Iniguez Huerta
// DATE:
//              03/21/2010
//============================================================================
`timescale 100ns / 10ns

module RS232TxRx(clk_50M, reset, rx_bit, tx_bit, leds_out);
  input        clk_50M;           // FPGA clock
  input        reset;             // FPGA reset
  input        rx_bit;            // input bit
  output       tx_bit;            // bit to transmit
  output [7:0] leds_out;          // bits to be directed to the FPGA leds
  
  // DATA BIT STATES
  reg [4:0] NXT_STATE;

  reg [23:0] rLedSwitchTime;
  reg rStart;
  reg [7:0] rTxByte;


// wires
  wire wParityError;
  wire [7:0] wRxByte;
  wire wValidOut;
  wire wDone;
  wire wTxBit;
  
  
  // declare the Data bit states
  parameter WAIT_BYTE             = 5'b00001,
            TRANSMIT_BYTE         = 5'b00010,
            TRANSMITTING_BYTE     = 5'b00100,
            PARITY_ERROR_LEDS_ON  = 5'b01000,
            PARITY_ERROR_LEDS_OFF = 5'b10000;

  /////////////////////////////////////////////////////////////////
  // Process the resolved bit
  /////////////////////////////////////////////////////////////////
  always @(posedge clk_50M, posedge reset)
  begin
    if(reset)
    begin
      rLedSwitchTime <= 24'b0;
      NXT_STATE <= WAIT_BYTE;
		  rTxByte <= 8'b10101010;
    end
    else
    begin
      // if a whole 1 byte was received
      if (wValidOut)
      begin
        if(!wParityError)
        begin
          // return same character
          rTxByte <= wRxByte;

          // if lower case letter received, convert it to uppercase before returning
          if (wRxByte > 96 && wRxByte < 123)
             rTxByte <= wRxByte - 32;
          // if upper case letter received, convert it to lowercase
          else if (wRxByte > 64 && wRxByte < 91)
             rTxByte <= wRxByte + 32;
        end
      end
      
      case (NXT_STATE)
        WAIT_BYTE:
        begin 
          rStart <= 0;
  
          // if a whole 1 byte was received
          if (wValidOut)
          begin
            if(!wParityError)
              // change state to read for more bits
              NXT_STATE <= TRANSMIT_BYTE;
            else  // parity, Error
              // change state to read for more bits
              NXT_STATE <= PARITY_ERROR_LEDS_ON;
          end
        end
        TRANSMIT_BYTE:
        begin
          rStart <= 1;
          NXT_STATE <= TRANSMITTING_BYTE;
        end
        TRANSMITTING_BYTE:
        begin
          rStart <= 0;
  
          if(wDone)
            NXT_STATE <= WAIT_BYTE;
        end
        PARITY_ERROR_LEDS_ON:
        begin
          rLedSwitchTime <= rLedSwitchTime + 1;
  
          // if a whole 1 byte was received
          if (wValidOut)
          begin
            if(!wParityError)
              // change state to read for more bits
              NXT_STATE <= TRANSMIT_BYTE;
          end
          else
          begin
            // if parityError, turn on all leds
            rTxByte <= 8'b11111111;
  
            if(rLedSwitchTime[23])
            begin
              rLedSwitchTime <= 0;
              NXT_STATE <= PARITY_ERROR_LEDS_OFF;
            end
          end
        end
        PARITY_ERROR_LEDS_OFF:
        begin
          rLedSwitchTime <= rLedSwitchTime + 1;
  
          // if a whole 1 byte was received
          if (wValidOut)
          begin
            if(!wParityError)
              // change state to read for more bits
              NXT_STATE <= TRANSMIT_BYTE;
          end
          else
          begin
            // if parityError, turn on all leds
            rTxByte <= 8'b00000000;
  
            if(rLedSwitchTime[23])
            begin
              rLedSwitchTime <= 0;
              NXT_STATE <= PARITY_ERROR_LEDS_ON;
            end
          end
        end
        default:
        begin
          NXT_STATE <= WAIT_BYTE;
        end
      endcase
    end
  end

  assign leds_out = rTxByte;
  assign tx_bit = wTxBit;

  UARTRx  UARTRx_inst5(
  .clk_50M(clk_50M),                    // FPGA clock
  .reset(reset),
  .data_in(rx_bit),                     // input bit vector
  .data_out(wRxByte),                   // parallel output bit vector
  .valid_out(wValidOut),                // indicates if a full byte was received
  .parity_error_out(wParityError));     // indicates if there is a parity error 

  UARTTx  UARTTx_inst6(
  .clk_50M(clk_50M),                    // FPGA clock
  .reset(reset),
  .data(rTxByte),                     // data to send
 .start(rStart),                      // indicator of start of transmision
  .tx_bit(wTxBit),                      // bit to transmit
  .done(wDone));                        // indicates the byte was transmitted 

endmodule




#include "nrf24.h"

/**
 * @brief Đọc giá trị 1 thanh ghi
 *
 * @param reg Địa chỉ của thanh ghi cần đọc
 * @return uint8_t Giá trị của thanh ghi
 */
static uint8_t nRF24_ReadReg(uint8_t reg)
{
	uint8_t value;

	nRF24_CSN_L();
	nRF24_LL_RW(reg & nRF24_MASK_REG_MAP);
	value = nRF24_LL_RW(nRF24_CMD_NOP);
	nRF24_CSN_H();

	return value;
}

/**
 * @brief Viết 1 byte vào thanh ghi
 *
 * @param reg Địa chỉ của thanh ghi
 * @param value Giá trị cần viết
 */
static void nRF24_WriteReg(uint8_t reg, uint8_t value)
{
	nRF24_CSN_L();
	if (reg < nRF24_CMD_W_REGISTER)
	{
		// This is a register access
		nRF24_LL_RW(nRF24_CMD_W_REGISTER | (reg & nRF24_MASK_REG_MAP));
		nRF24_LL_RW(value);
	}
	else
	{
		// This is a single byte command or future command/register
		nRF24_LL_RW(reg);
		if ((reg != nRF24_CMD_FLUSH_TX) && (reg != nRF24_CMD_FLUSH_RX) &&
			(reg != nRF24_CMD_REUSE_TX_PL) && (reg != nRF24_CMD_NOP))
		{
			// Send register value
			nRF24_LL_RW(value);
		}
	}
	nRF24_CSN_H();
}

/**
 * @brief Đọc nhiều byte từ thanh ghi
 *
 * @param reg Địa chỉ thanh ghi
 * @param pBuf Con trỏ , trỏ vào buffer cần lưu
 * @param count Số lượng byte
 */
static void nRF24_ReadMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count)
{
	nRF24_CSN_L();
	nRF24_LL_RW(reg);
	while (count--)
	{
		*pBuf++ = nRF24_LL_RW(nRF24_CMD_NOP);
	}
	nRF24_CSN_H();
}

/**
 * @brief Viết nhiều byte vào thanh ghi
 *
 * @param reg Địa chỉ của thanh ghi
 * @param pBuf Trỏ đến buffer lưu giá trị
 * @param count số lượng byte
 */
static void nRF24_WriteMBReg(uint8_t reg, uint8_t *pBuf, uint8_t count)
{
	nRF24_CSN_L();
	nRF24_LL_RW(reg);
	while (count--)
	{
		nRF24_LL_RW(*pBuf++);
	}
	nRF24_CSN_H();
}

/**
 * @brief Init bộ Transmit và Receive
 * note: RX/TX chưa cấu hình đại chỉ pipe
 *
 */
void nRF24_Init(void)
{

	nRF24_WriteReg(nRF24_REG_CONFIG, 0x08);		// Bật tính CRC
	nRF24_WriteReg(nRF24_REG_EN_AA, 0x3F);		// Cho phép phản hồi ACK cho pipe0 -> pipe5
	nRF24_WriteReg(nRF24_REG_EN_RXADDR, 0x03);	// Cho phép truyền pipe0 và pipe1
	nRF24_WriteReg(nRF24_REG_SETUP_AW, 0x03);	// Độ dài địa chỉ là 5 byte
	nRF24_WriteReg(nRF24_REG_SETUP_RETR, 0x03); // 0000 0011 , Cho phép gửi lại 3 lần(reTransmit) và mỗi lần gửi cách nhau 250us
	nRF24_WriteReg(nRF24_REG_RF_CH, 0x02);		// 2400 + 2 (hz)
	nRF24_WriteReg(nRF24_REG_RF_SETUP, 0x0E);	// tốc độ truyền 2Mbps , và công suất độ lợi tối đa 0dm
	nRF24_WriteReg(nRF24_REG_STATUS, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P0, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P1, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P2, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P3, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P4, 0x00);
	nRF24_WriteReg(nRF24_REG_RX_PW_P5, 0x00);
	nRF24_WriteReg(nRF24_REG_DYNPD, 0x00);
	nRF24_WriteReg(nRF24_REG_FEATURE, 0x00);

	// Clear the FIFO's
	nRF24_FlushRX(); //  xóa bộ đệm RX
	nRF24_FlushTX(); //  xóa bộ đệm TX

	// Xóa hết những sự kiện vào ngắt
	nRF24_ClearIRQFlags();

	// (chip release)
	nRF24_CSN_H();
}

/**
 * @brief Kiểm tra nRF24L01 có hoạt động hay không
 *
 * @return 1 - nRF24L01 đang hoạt động
 * @return 0 - No đéo hoạt động
 *
 */
uint8_t nRF24_Check(void)
{
	uint8_t rxbuf[5];
	uint8_t i;
	uint8_t *ptr = (uint8_t *)nRF24_TEST_ADDR;

	// Viết Địa chỉ giả để test TX và RX
	nRF24_WriteMBReg(nRF24_CMD_W_REGISTER | nRF24_REG_TX_ADDR, ptr, 5);
	nRF24_ReadMBReg(nRF24_CMD_R_REGISTER | nRF24_REG_TX_ADDR, rxbuf, 5);

	// So sánh hai giá trị , sai thì return 0
	for (i = 0; i < 5; i++)
	{
		if (rxbuf[i] != *ptr++)
			return 0;
	}

	return 1;
}

/**
 * @brief Điều chình transceiver power mode
 *
 * @param mode trạng thát mới của power mode , Power up và Power down , tương ứng TX hay RX
 */
void nRF24_SetPowerMode(uint8_t mode)
{
	uint8_t reg;

	reg = nRF24_ReadReg(nRF24_REG_CONFIG);
	if (mode == nRF24_PWR_UP)
	{
		// kích hoạt transmit
		reg |= nRF24_CONFIG_PWR_UP;
	}
	else
	{
		// kích hoạt receive
		reg &= ~nRF24_CONFIG_PWR_UP;
	}
	nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

/**
 * @brief et chế động hoạt động
 *
 * @param mode mode hoạt động , TX hay RX
 */
void nRF24_SetOperationalMode(uint8_t mode)
{
	uint8_t reg;

	// cấu PRIM_RX bit để chọn chế độ
	reg = nRF24_ReadReg(nRF24_REG_CONFIG);
	reg &= ~nRF24_CONFIG_PRIM_RX;
	reg |= (mode & nRF24_CONFIG_PRIM_RX);
	nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

/**
 * @brief Cài đặt có thể thay đổi độ dài payload ()
 *
 * @param mode status, one of nRF24_DPL_xx value
 */
void nRF24_SetDynamicPayloadLength(uint8_t mode)
{
	uint8_t reg;
	reg = nRF24_ReadReg(nRF24_REG_FEATURE);
	if (mode)
	{
		nRF24_WriteReg(nRF24_REG_FEATURE, reg | nRF24_FEATURE_EN_DPL); // set thanh ghi Feature (enable payload các kiểu)
		nRF24_WriteReg(nRF24_REG_DYNPD, 0x1F);						   //  Set tất cả các pipe đều có thể thyad dổi payload
	}
	else
	{
		nRF24_WriteReg(nRF24_REG_FEATURE, reg & ~nRF24_FEATURE_EN_DPL); //  hủy thay đổi payload
		nRF24_WriteReg(nRF24_REG_DYNPD, 0x0);
	}
}

/**
 * @brief CHo phép gửi ACK với thay đổi payload.
 *
 * @param mode status, 1 or 0
 */
void nRF24_SetPayloadWithAck(uint8_t mode)
{
	uint8_t reg;
	reg = nRF24_ReadReg(nRF24_REG_FEATURE);
	if (mode)
	{
		nRF24_WriteReg(nRF24_REG_FEATURE, reg | nRF24_FEATURE_EN_ACK_PAY);
	}
	else
	{
		nRF24_WriteReg(nRF24_REG_FEATURE, reg & ~nRF24_FEATURE_EN_ACK_PAY);
	}
}

/**
 * @brief Cơ chế chắc CRC
 *
 * @param scheme Cơ chế CRC
 * @note: Bộ transReceier BẮT BUỘC bật CRC nếu có ít nhất một cổng pipe bật autoACK
 */
void nRF24_SetCRCScheme(uint8_t scheme)
{
	uint8_t reg;

	// Configure EN_CRC[3] and CRCO[2] bits of the CONFIG register
	reg = nRF24_ReadReg(nRF24_REG_CONFIG);
	reg &= ~nRF24_MASK_CRC;
	reg |= (scheme & nRF24_MASK_CRC);
	nRF24_WriteReg(nRF24_REG_CONFIG, reg);
}

/**
 * @brief Sét tần số Hoạt động
 *
 * @param channel Giá trị tần số 0 to 127
 * @Note: Tần số  =  (2400 + channel)MHz
 * @Note: PLOS_CNT[7:4] bits of the OBSERVER_TX register will be rese
 */
void nRF24_SetRFChannel(uint8_t channel)
{
	nRF24_WriteReg(nRF24_REG_RF_CH, channel);
}

// Set tham số tự động truyền ReTrasmit
// input:
//   ard -Thời gian giữa những lần truyền lại
//   arc - Số lần truyện lại (tối đa 15)
// note: 0x00 có nghĩa là tắt chức năng truyền lại
void nRF24_SetAutoRetr(uint8_t ard, uint8_t arc)
{
	nRF24_WriteReg(nRF24_REG_SETUP_RETR, (uint8_t)((ard << 4) | (arc & nRF24_MASK_RETR_ARC)));
}

/**
 * @brief Set Chiều dài của địa chỉ
 *
 * @param addr_width addr_width - độ dài RX/TX address (3 to 5)
 * @Note: cài đặt cho tất cả các pipe
 */
void nRF24_SetAddrWidth(uint8_t addr_width)
{
	nRF24_WriteReg(nRF24_REG_SETUP_AW, addr_width - 2);
}

/**
 * @brief Set địa chỉ cho pipe
 *
 * @param pipe pipe cần cấu hình
 * @param addr pointer to the buffer with address
 * @note: pipe có giá trị 0 to 5 (RX pipes) và 6 (TX pipe)
 * @note: buffer length must be equal to current address width of transceiver
 * @note: Với pipes[2..5] Chỉ có byte đầu tên được GHI vì những byte còn lại giống pipe1
 */
void nRF24_SetAddr(uint8_t pipe, const uint8_t *addr)
{
	uint8_t addr_width;

	// RX_ADDR_Px register
	switch (pipe)
	{
	case nRF24_PIPETX:
	case nRF24_PIPE0:
	case nRF24_PIPE1:
		// Lấy độ dài address
		addr_width = nRF24_ReadReg(nRF24_REG_SETUP_AW) + 1;

		addr += addr_width;
		nRF24_CSN_L();
		nRF24_LL_RW(nRF24_CMD_W_REGISTER | nRF24_ADDR_REGS[pipe]);
		do
		{
			nRF24_LL_RW(*addr--);
		} while (addr_width--);
		nRF24_CSN_H();
		break;
	case nRF24_PIPE2:
	case nRF24_PIPE3:
	case nRF24_PIPE4:
	case nRF24_PIPE5:
		// Chỉ viết byte đầu tiên (vì những byte còn lại gióng pipe1)
		nRF24_WriteReg(nRF24_ADDR_REGS[pipe], *addr);
		break;
	default:
		break;
	}
}
/**
 * @brief Cấu hình TX mode
 *
 * @param tx_pwr Công suất của RF , (0db,1db ,2db)
 */
void nRF24_SetTXPower(uint8_t tx_pwr)
{
	uint8_t reg;

	reg = nRF24_ReadReg(nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_RF_PWR;
	reg |= tx_pwr;
	nRF24_WriteReg(nRF24_REG_RF_SETUP, reg);
}
/**
 * @brief Cấu hinhd RX mode
 *
 * @param data_rate Tốc độ của data
 */
void nRF24_SetDataRate(uint8_t data_rate)
{
	uint8_t reg;

	reg = nRF24_ReadReg(nRF24_REG_RF_SETUP);
	reg &= ~nRF24_MASK_DATARATE;
	reg |= data_rate;
	nRF24_WriteReg(nRF24_REG_RF_SETUP, reg);
}
/**
 * @brief Cấu hình Những pipe cụ thể
 *
 * @param pipe Pipe cần cấu hình
 * @param aa_state Bật phản hồi ACK hay không (nRF24_AA_ON or nRF24_AA_OFF)
 * @param payload_len độ dài của payload
 */
void nRF24_SetRXPipe(uint8_t pipe, uint8_t aa_state, uint8_t payload_len)
{
	uint8_t reg;

	// Enable pipe (EN_RXADDR register)
	reg = (nRF24_ReadReg(nRF24_REG_EN_RXADDR) | (1 << pipe)) & nRF24_MASK_EN_RX;
	nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg);

	// Set RX payload length (RX_PW_Px register)
	nRF24_WriteReg(nRF24_RX_PW_PIPE[pipe], payload_len & nRF24_MASK_RX_PW);

	// Set auto acknowledgment
	reg = nRF24_ReadReg(nRF24_REG_EN_AA);
	if (aa_state == nRF24_AA_ON)
	{
		reg |= (1 << pipe);
	}
	else
	{
		reg &= ~(1 << pipe);
	}
	nRF24_WriteReg(nRF24_REG_EN_AA, reg);
}

/**
 * @brief Disable specified RX pipe
 *
 * @param pipe Pipe cần cấu hình
 */
void nRF24_ClosePipe(uint8_t pipe)
{
	uint8_t reg;

	reg = nRF24_ReadReg(nRF24_REG_EN_RXADDR);
	reg &= ~(1 << pipe);
	reg &= nRF24_MASK_EN_RX;
	nRF24_WriteReg(nRF24_REG_EN_RXADDR, reg);
}

/**
 * @brief CHo phép auto retransmit cho từng pipe
 *
 * @param pipe pipe cần cấu hình
 */
void nRF24_EnableAA(uint8_t pipe)
{
	uint8_t reg;

	// Set bit in EN_AA register
	reg = nRF24_ReadReg(nRF24_REG_EN_AA);
	reg |= (1 << pipe);
	nRF24_WriteReg(nRF24_REG_EN_AA, reg);
}
/**
 * @brief Hủy phản hồi ACK cho 1 pipe or tất cả các pipe
 *
 * @param pipe số pipe cần cấu hình
 */
void nRF24_DisableAA(uint8_t pipe)
{
	uint8_t reg;

	if (pipe > 5)
	{
		// Disable Auto-ACK cho ALL pipes
		nRF24_WriteReg(nRF24_REG_EN_AA, 0x00);
	}
	else
	{
		// Clear bit in the EN_AA register
		reg = nRF24_ReadReg(nRF24_REG_EN_AA);
		reg &= ~(1 << pipe);
		nRF24_WriteReg(nRF24_REG_EN_AA, reg);
	}
}

/**
 * @brief lấy giá trị STATUS register
 *
 * @return uint8_t Giá trị của STATUS
 */
uint8_t nRF24_GetStatus(void)
{
	return nRF24_ReadReg(nRF24_REG_STATUS);
}

/**
 * @brief Lấy Những Pending của IRQ
 *
 * @return uint8_t giá trị của of RX_DR, TX_DS and MAX_RT trong STATUS register
 */
uint8_t nRF24_GetIRQFlags(void)
{
	return (nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_STATUS_IRQ);
}

/**
 * @brief Lấy tastus RX FIFO
 *
 * @return uint8_t one of the nRF24_STATUS_RXFIFO_xx values
 */
uint8_t nRF24_GetStatus_RXFIFO(void)
{
	return (nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_RXFIFO);
}
/**
 * @brief Lấy status TX FIFO
 *
 * @return uint8_t one of the nRF24_STATUS_TXFIFO_xx values
 * note: TX_REUSE bit được bỏ qua
 */
uint8_t nRF24_GetStatus_TXFIFO(void)
{
	return ((nRF24_ReadReg(nRF24_REG_FIFO_STATUS) & nRF24_MASK_TXFIFO) >> 4);
}

// Get pipe number for the payload available for reading from RX FIFO
// return: pipe number or 0x07 if the RX FIFO is empty
uint8_t nRF24_GetRXSource(void)
{
	return ((nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1);
}

// Get auto retransmit statistic
// return: value of OBSERVE_TX register which contains two counters encoded in nibbles:
//   high - lost packets count (max value 15, can be reseted by write to RF_CH register)
//   low  - retransmitted packets count (max value 15, reseted when new transmission starts)
uint8_t nRF24_GetRetransmitCounters(void)
{
	return (nRF24_ReadReg(nRF24_REG_OBSERVE_TX));
}

/**
 * @brief Reset packet lost counter (PLOS_CNT bits in OBSERVER_TX register)
 *
 */
void nRF24_ResetPLOS(void)
{
	uint8_t reg;

	// The PLOS counter is reset after write to RF_CH register
	reg = nRF24_ReadReg(nRF24_REG_RF_CH);
	nRF24_WriteReg(nRF24_REG_RF_CH, reg);
}

/**
 * @brief xóa TX FIFO
 *
 */
void nRF24_FlushTX(void)
{
	nRF24_WriteReg(nRF24_CMD_FLUSH_TX, nRF24_CMD_NOP);
}

/**
 * @brief xóa RX FIFO
 *
 */
void nRF24_FlushRX(void)
{
	nRF24_WriteReg(nRF24_CMD_FLUSH_RX, nRF24_CMD_NOP);
}

/**
 * @brief xóa bất kỳ pending IRQ flags
 *
 */
void nRF24_ClearIRQFlags(void)
{
	uint8_t reg;

	// Clear RX_DR, TX_DS and MAX_RT bits của thanh STATUS
	reg = nRF24_ReadReg(nRF24_REG_STATUS);
	reg |= nRF24_MASK_STATUS_IRQ;
	nRF24_WriteReg(nRF24_REG_STATUS, reg);
}
/**
 * @brief Ghi TX payload
 *
 * @param pBuf con trỏ , trỏ tới buffer chứa paylaod
 * @param length độ dài của paylaod
 */
void nRF24_WritePayload(uint8_t *pBuf, uint8_t length)
{
	nRF24_WriteMBReg(nRF24_CMD_W_TX_PAYLOAD, pBuf, length);
}
/**
 * @brief đọc chiều dài của RX payload
 *
 * @return uint8_t độ dài
 */
static uint8_t nRF24_GetRxDplPayloadWidth()
{
	uint8_t value;

	nRF24_CSN_L();
	nRF24_LL_RW(nRF24_CMD_R_RX_PL_WID);
	value = nRF24_LL_RW(nRF24_CMD_NOP);
	nRF24_CSN_H();

	return value;
}

static nRF24_RXResult nRF24_ReadPayloadGeneric(uint8_t *pBuf, uint8_t *length, uint8_t dpl)
{
	uint8_t pipe;

	// Extract a payload pipe number from the STATUS register
	pipe = (nRF24_ReadReg(nRF24_REG_STATUS) & nRF24_MASK_RX_P_NO) >> 1;

	// RX FIFO empty?
	if (pipe < 6)
	{
		// Get payload length
		if (dpl)
		{
			*length = nRF24_GetRxDplPayloadWidth();
			if (*length > 32)
			{ // broken packet
				*length = 0;
				nRF24_FlushRX();
			}
		}
		else
		{
			*length = nRF24_ReadReg(nRF24_RX_PW_PIPE[pipe]); // đọc payload của pipe x
		}

		// đọc payload từ RX FIFO
		if (*length)
		{
			nRF24_ReadMBReg(nRF24_CMD_R_RX_PAYLOAD, pBuf, *length);
		}

		return ((nRF24_RXResult)pipe);
	}

	// The RX FIFO is empty
	*length = 0;

	return nRF24_RX_EMPTY;
}
/**
 * @brief Read top level payload available in the RX FIFO
 *
 * @param pBuf COn trỏ lưu data
 * @param length con trỏ lưu độ dài
 * @return   nRF24_RX_PIPEX - đã nhận được dữ liệu từ pipe x
 * @return	 nRF24_RX_EMPTY - the RX FIFO đéo có cc gì
 *
 */
nRF24_RXResult nRF24_ReadPayload(uint8_t *pBuf, uint8_t *length)
{
	return nRF24_ReadPayloadGeneric(pBuf, length, 0);
}

nRF24_RXResult nRF24_ReadPayloadDpl(uint8_t *pBuf, uint8_t *length)
{
	return nRF24_ReadPayloadGeneric(pBuf, length, 1);
}

uint8_t nRF24_GetFeatures()
{
	return nRF24_ReadReg(nRF24_REG_FEATURE);
}
void nRF24_ActivateFeatures()
{
	nRF24_CSN_L();
	nRF24_LL_RW(nRF24_CMD_ACTIVATE);
	nRF24_LL_RW(0x73);
	nRF24_CSN_H();
}
void nRF24_WriteAckPayload(nRF24_RXResult pipe, char *payload, uint8_t length)
{
	nRF24_CSN_L();
	nRF24_LL_RW(nRF24_CMD_W_ACK_PAYLOAD | pipe);
	while (length--)
	{
		nRF24_LL_RW((uint8_t)*payload++);
	}
	nRF24_CSN_H();
}

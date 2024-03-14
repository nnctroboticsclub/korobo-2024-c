RoboProtocol = Proto.new("robo", "Robo Protocol")
RoboProtocol.fields.length = ProtoField.uint16("robo.length", "Length")
RoboProtocol.fields.opcode = ProtoField.uint8("robo.opcode", "opcode")
RoboProtocol.fields.payload = ProtoField.bytes("robo.payload", "Payload")

function RoboProtocol.dissector(buffer, pinfo, tree)
    local buffer_end = buffer:len()
    local message_count = 0
    local ptr = 0

    while true do
        local packet_top = ptr

        if buffer_end - ptr < 4 then
            ptr = packet_top
            break
        end
        local payload_size = buffer:range(ptr, 4)
        ptr = ptr + 4


        if buffer_end - ptr < payload_size:uint() then
            ptr = packet_top
            break
        end
        local payload = buffer:range(ptr, payload_size:uint())
        ptr = ptr + payload_size:uint()

        opcode = payload:range(0, 1)
        data = payload:range(1)

        local packet_range = buffer:range(packet_top, 4 + payload_size:uint() - 1)
        local subtree = tree:add(RoboProtocol, packet_range)
        subtree:add(RoboProtocol.fields.length, payload_size)
        subtree:add(RoboProtocol.fields.opcode, opcode)
        subtree:add(RoboProtocol.fields.payload, data)
        subtree:set_text("Robo (" .. opcode:bytes():tohex() .. ": " .. payload:bytes():tohex() .. ")")


        ptr = ptr + 4 + payload_size:uint()
        message_count = message_count + 1
    end

    pinfo.cols.protocol = "Robo"
    if message_count == 1 then
        pinfo.cols.info = opcode:bytes():tohex() .. ": " .. data:bytes():tohex()
    else
        pinfo.cols.info = "Packets: " .. message_count .. "."
    end
end

udp_table = DissectorTable.get("udp.port")
udp_table:add(8001, RoboProtocol)
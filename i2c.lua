-- I2C Scanner für ArduPilot Lua
-- Scannt Bus 0 und 1 (passe dies ggf. an)

local busse_zum_scannen = {0, 1} 

function update()
  gcs:send_text(6, "Starte I2C Scan...") -- 6 = MAV_SEVERITY_INFO

  for _, bus in ipairs(busse_zum_scannen) do
    gcs:send_text(6, "Scanne Bus: " .. tostring(bus))
    
    for adresse = 1, 127 do
      -- Versuche, das Gerät zu pingen (dies ist eine vereinfachte Methode)
      -- ArduPilot Lua hat keine direkte "Ping"-Funktion, aber wir können
      -- versuchen, ein generisches Device-Handle zu bekommen.
      
      local dev = i2c:get_device(bus, adresse)
      
      if dev then
        -- Um wirklich zu wissen, ob es da ist, müsste man oft ein Register lesen.
        -- Wenn get_device nil zurückgibt, ist gar nichts definiert.
        -- Hinweis: Dies zeigt oft nur konfigurierte Treiber an.
        
        -- Ein echter "Blind Scan" ist in Lua eingeschränkt, um Bus-Hänger zu vermeiden.
        -- Die sicherste Methode ist meist der MAVProxy Ansatz (siehe unten).
      end
    end
  end

  return update, 10000 -- Wiederhole alle 10 Sekunden
end

return update()
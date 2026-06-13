--[[
 TruckPilot Nav-Route RE — Cheat Engine Lua (1.59)
 Fixes gegen Original:
   - soValueBetween statt soExactValue fuer Bereichs-Scan
   - Phase 0: nav_distance (Restdistanz, faellt beim Fahren) statt trip_distance
   - trip_distance = Gesamtlaenge, Offset +0x21C evtl. 1.59-veraendert
]]

local OFF = {
  simple_route_source = 0x08,
  route_task          = 0x20,
  phys_items          = 0x50,
  item_stride         = 0x20,
  item_node           = 0x00,
  item_dist_left      = 0x14,
  node_uid            = 0x30,
  gps_trip_distance   = 0x21C,
  gps_trip_time       = 0x220,
}

local function hexs(v)
  if v == nil then return "nil" end
  return string.format("0x%X", v)
end

local function looksLikePointer(v)
  return v ~= nil and v > 0x10000 and v < 0x7FFFFFFFFFFF
end

-- Phase 0: Restdistanz (GPS-Anzeige, sinkt beim Fahren) — EINFACHER Einstieg
function tp_scan_navdist(meters)
  meters = meters or 50000
  local lo = meters - 2000
  local hi = meters + 2000
  print(string.format("[Phase0] Scanne Float (Restdistanz) %d..%d m ...", lo, hi))
  print("[Phase0] Tipp: Wert aus GPS-Anzeige oder TruckPilot-SHM (scripts/read_shm_nav.py)")

  local ms = getCurrentMemscan and getCurrentMemscan() or createMemScan()
  ms.firstScan(
    soValueBetween,   -- FIX: war soExactValue (liefert 0 Treffer bei Bereich!)
    vtSingle,
    rtRounded,
    tostring(lo),
    tostring(hi),
    0, 0x7fffffffffffffff,
    "*X*",            -- alle committed (nicht nur writable)
    fsmNotAligned,
    "4",
    true, false, false, false
  )
  ms.waitTillDone()
  local fl = createFoundList(ms)
  fl.initialize()
  print(string.format("[Phase0] Treffer: %d", fl.Count))
  print("[Phase0] Jetzt 500m fahren, dann tp_filter_decreased()")
  _tp_ms = ms
  _tp_fl = fl
end

function tp_filter_decreased()
  if _tp_ms == nil then print("Erst tp_scan_navdist() aufrufen.") return end
  _tp_ms.nextScan(soDecreasedValue, vtSingle, rtRounded, "", "", 0,0, "", fsmNotAligned, "4", true,false,false,false)
  _tp_ms.waitTillDone()
  if _tp_fl then _tp_fl.deinitialize() end
  _tp_fl = createFoundList(_tp_ms)
  _tp_fl.initialize()
  print(string.format("[Phase0b] Nach Decreased: %d Treffer", _tp_fl.Count))
  if _tp_fl.Count <= 20 then tp_show_candidates() end
end

-- Phase 1: trip_distance (Gesamtlaenge, UNVERAENDERT beim Fahren)
function tp_scan_tripdist(meters)
  meters = meters or 50000
  local lo = meters - 5000
  local hi = meters + 5000
  print(string.format("[Phase1] Scanne trip_distance (Gesamt) %d..%d m ...", lo, hi))

  local ms = getCurrentMemscan and getCurrentMemscan() or createMemScan()
  ms.firstScan(
    soValueBetween,   -- FIX
    vtSingle,
    rtRounded,
    tostring(lo),
    tostring(hi),
    0, 0x7fffffffffffffff,
    "*X*",
    fsmNotAligned,
    "4",
    true, false, false, false
  )
  ms.waitTillDone()
  local fl = createFoundList(ms)
  fl.initialize()
  print(string.format("[Phase1] Treffer: %d", fl.Count))
  _tp_ms = ms
  _tp_fl = fl
end

function tp_filter_unchanged()
  if _tp_ms == nil then print("Erst tp_scan_tripdist() aufrufen.") return end
  _tp_ms.nextScan(soUnchanged, vtSingle, rtRounded, "", "", 0,0, "", fsmNotAligned, "4", true,false,false,false)
  _tp_ms.waitTillDone()
  if _tp_fl then _tp_fl.deinitialize() end
  _tp_fl = createFoundList(_tp_ms)
  _tp_fl.initialize()
  print(string.format("[Phase1b] Stabile Treffer: %d", _tp_fl.Count))
  if _tp_fl.Count <= 12 then tp_show_candidates() end
end

function tp_show_candidates()
  if _tp_fl == nil then print("Keine Trefferliste.") return end
  print(string.format("[Kandidaten] %d Treffer:", _tp_fl.Count))
  local n = math.min(_tp_fl.Count, 30)
  for i=0, n-1 do
    local addr = _tp_fl.Address[i]
    local val  = readFloat("0x"..addr)
    local gps_base = tonumber(addr, 16) - OFF.gps_trip_distance
    print(string.format("  %2d  addr=0x%s val=%s  (gps_base wenn +0x21C: %s)",
      i, addr, tostring(val), hexs(gps_base)))
  end
end

function tp_probe(base)
  if type(base) == "string" then base = tonumber(base) end
  print(string.format("[Probe] Struktur ab %s", hexs(base)))
  for off=0,0x280,0x08 do
    local q = readQword(base+off)
    local f = readFloat(base+off)
    local tag = looksLikePointer(q) and " <ptr?>" or ""
    if off <= 0x60 or off == OFF.gps_trip_distance or off == OFF.gps_trip_time then
      print(string.format("  +0x%03X  q=%-18s  f=%-14s%s", off, hexs(q), tostring(f), tag))
    end
  end
end

function tp_walk(gps)
  if type(gps) == "string" then gps = tonumber(gps) end
  print(string.format("[Walk] gps_manager = %s", hexs(gps)))
  local td = readFloat(gps + OFF.gps_trip_distance)
  print(string.format("[Walk] +0x21C trip_distance = %s", tostring(td)))

  local route_task = readQword(gps + OFF.simple_route_source + OFF.route_task)
  print(string.format("[Walk] route_task = %s", hexs(route_task)))
  if not looksLikePointer(route_task) then return end

  local arr_ptr  = readQword(route_task + OFF.phys_items + 0x00)
  local arr_size = readQword(route_task + OFF.phys_items + 0x08)
  local arr_cap  = readQword(route_task + OFF.phys_items + 0x10)
  print(string.format("[Walk] items ptr=%s size=%s cap=%s", hexs(arr_ptr), tostring(arr_size), tostring(arr_cap)))
  if not looksLikePointer(arr_ptr) or arr_size == nil or arr_size == 0 or arr_size > 6000 then return end

  local uids = {}
  local maxshow = math.min(arr_size, 2000)
  for i=0, maxshow-1 do
    local item = arr_ptr + i*OFF.item_stride
    local node = readQword(item + OFF.item_node)
    local dist = readFloat(item + OFF.item_dist_left)
    if looksLikePointer(node) then
      local uid = readQword(node + OFF.node_uid)
      table.insert(uids, {uid=uid, dist=dist})
      if i < 5 or i >= maxshow-2 then
        print(string.format("  item[%d] node=%s uid=%s dist_left=%s", i, hexs(node), tostring(uid), tostring(dist)))
      end
    end
  end
  if #uids > 0 then
    print(string.format("[Walk] ERSTE UID = %s", tostring(uids[1].uid)))
    print(string.format("[Walk] LETZTE UID = %s", tostring(uids[#uids].uid)))
  end
end

print("TruckPilot Nav-RE Lua v2 geladen (1.59 fixes).")
print("Empfohlen: tp_scan_navdist(<Restdistanz_aus_GPS>) -> fahren -> tp_filter_decreased()")
print("Dann 'Find what accesses' auf Treffer -> gps_manager ableiten.")

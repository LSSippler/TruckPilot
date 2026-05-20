# TruckPilot Cockpit Design

_Started 2026-05-18 15:28 UTC_

---

## User

<system-info comment="Only acknowledge these if relevant">
Project title is now "TruckPilot UI"
Current date is now May 18, 2026
</system-info>

<default aesthetic>
If no references, art direction or design systems were provided, use this default professional modern, minimal aesthetic as a base. Declare your choice out loud so you stick to it. Guidance:
- Choose a type pairing from web-safe set or Google Fonts. Helvetica is a good choice. Avoid hard-to-read or overly stylized fonts. Use 1-3 fonts only.
- Foreground and background: choose a color tone (warm, cool, neutral, something in-between). Use subtly-toned whites and blacks; avoid saturations above 0.02 for whites.
- Accents: choose 0-2 additional accent colors using oklch. All accents should share same chroma and lightness; vary hue.
- NEVER write out an SVG yourself that's more complicated than a square, circle, diamond, etc.
- For imagery, never hand-draw SVGs; use subtly-striped SVG placeholders instead with monospace explainers for what should be dropped there (e.g. “product shot”)

CRITICAL: ignore default aesthetic entirely if given other aesthetic instructions like reference images, design systems or guidance, or if there are files in the project already.
</default aesthetic>

<attached_files>
- uploads/Screenshot 2026-05-18 162506.png
- uploads/Screenshot 2026-05-18 162509.png
- uploads/Screenshot 2026-05-18 162511.png
- uploads/Screenshot 2026-05-18 162513.png
- uploads/Screenshot 2026-05-18 162514.png
- uploads/truckpilot_icon.png
</attached_files>

<pasted_text name="Pasted text (307 lines)">
TruckPilot UI Design Polish

# Rolle

Du bist Senior Product Designer fuer ein technisches Self-Driving Tool. TruckPilot ist ein Desktop-Programm (Tauri + React + shadcn) das einen Truck in Euro Truck Simulator 2 autonom faehrt. Es zeigt Telemetry, Plugin-Status, vJoy-Output, Route-Daten und Logs in Echtzeit.

Aufgabe: Look and Feel komplett ueberarbeiten. Funktional ist die UI fertig, optisch wirkt sie wie ein generisches Admin-Dashboard. Sie soll wie ein hochwertiges Telemetry/Cockpit-Tool aussehen, das Spass macht zu benutzen und Vertrauen ausstrahlt waehrend man dem Programm die Kontrolle ueber einen 40-Tonner gibt.

# Routing-Architektur (wichtig fuer UI)

TruckPilot setzt KEINE eigene Route. Der User setzt das Ziel in der ETS2-Navigation (in-game). TruckPilot folgt der ETS2-Navi und kombiniert das mit den geparsten Map-Daten (.scs Sektoren, Node-Graph).

Das hat UI-Konsequenzen:
- KEIN Goal-UID Input als primary UI (kann als Diag/Override-Mode versteckt werden)
- Primary: Navi-Status anzeigen (next maneuver, distance, current road)
- Route-Visualisierung zeigt was die ETS2-Navi vorgibt, plus die TruckPilot-interpretierte Lane/Junction-Decision
- Engage Button braucht Precondition Navigation active in ETS2

# Logo als Design-Grundlage

Das Logo ist eine Lenkrad-Silhouette mit eingearbeitetem T, geometrisch konstruiert (Kreis + Dreieck-Anschnitt). Gradient laeuft von Teal/Mint Gruen ueber Cyan/Blue zu Purple/Violet, auf dunklem Hintergrund.

Daraus ableiten:

Brand-Gradient: Teal (#1ED891 oder aehnlich) -> Cyan -> Blue -> Purple. Nutzbar als Accent fuer Engaged-State, Active-Elements, Hero-Buttons, Sparkline-Gradienten.

Geometrie: das Logo ist konstruktiv (Kreis, Dreieck, klare Schnitte). UI-Elemente sollten geometrisch klar sein, klare Schnitte, keine fluffigen Rundungen. Mut zur Klarheit.

Dark-First: das Logo wirkt auf Schwarz/Dunkel am staerksten. Dark Theme ist Hero, Light Theme muss eigenstaendig funktionieren ohne das Brand-Gefuehl zu verlieren.

# Scope

Drei Ebenen, alle drei machen:

## 1. Visual Identity (Komplett-Redesign)

- Farbsystem aus dem Logo-Gradient ableiten:
  - Primary: Teal/Mint (Engaged, Success, Live-Werte)
  - Secondary: Cyan/Blue (Info, Active States)
  - Accent: Purple (Sekundaere Highlights, Sparkline-End-Glows)
  - Semantic: Warning (Amber), Error (Red, NICHT aus Gradient), Failsafe (Red mit Pulse)
  - Surfaces: 4-5 Dark-Layers (Base, Card, Elevated, Modal, Overlay), entsprechende Light-Layers
- Typography:
  - Display: fuer Hero-Numerals (Speed, RPM in grossem Engaged-Mode)
  - Heading-Hierarchie h1-h3
  - Body: regular UI Text
  - Mono: Telemetry-Werte, UIDs, Logs, Coordinates (Werte muessen ausgerichtet bleiben)
  - Vorschlag: Inter oder Geist fuer UI, JetBrains Mono oder Geist Mono fuer Werte
- Spacing-System: 4/8/12/16/24/32/48 Skala
- Card-Style: dunkle Surface mit subtiler Border (1px, semi-transparent), kein heavy Shadow im Dark Theme, im Light Theme dezente Shadows
- Animations:
  - Engage Button Pulse wenn Engaged
  - Value-Change Highlight (Speed-Update flasht kurz auf)
  - Card-Hover Lift sehr subtil
  - Page-Transition Cross-Fade unter 150ms
  - Sparkline Live-Pulse rechts (Dot mit Glow)
- Beide Themes:
  - Dark als Hero, Logo-Gradient kommt voll zur Geltung
  - Light bewusst gestaltet, nicht Color-Invert. Helle Surfaces aber Akzente vom Brand-Gradient erhalten

## 2. Information Architecture / Layout

Dashboard ist wichtigste Page. Cards (Autopilot, Telemetry, Navi-Status, Cruise, vJoy) brauchen klare Hierarchie. Refine:

Primary Action: Engage Button. Hero-Element, groesste Stelle im Dashboard, sofort sichtbar.

Status auf einen Blick: Preconditions, Active/Engaged State, ETS2-Navigation Active. Eng am Engage Button.

Live-Daten: Speed, RPM, Navi-Status (next maneuver + distance), vJoy-Output. Sekundaere Hierarchie, aber gut sichtbar wenn Autopilot aktiv ist.

Sekundaer: Konfiguration, Detail-Tabellen. Unten oder in Tabs.

Layout-Vorschlag erarbeiten mit Wireframes (in Markdown beschrieben oder als HTML-Mock):
- Hero-Zone oben (Engage + Status + Navi-Hint)
- Live-Zone Mitte (Telemetry + vJoy + Route-Progress)
- Detail-Zone unten oder Tab-System

Sidebar Gruppierung:
- Operation: Dashboard, Blackboard, Logs
- Configuration: Plugins, PID Tuning, Mods, Settings
Mit visueller Trennung (Header-Label oder Separator).

## 3. Komponenten-Polish

Existing Components systematisch:

Engage Button (Hero-Element):
- States: Off (ready), Off (preconditions missing, disabled), Engaging (Loading-Pulse), Engaged (Live-Pulse mit Brand-Gradient), Disengaging, Fault (Red)
- Gross, gradient bei Engaged, klar disabled wenn nicht ready
- Hotkey-Indicator (F5/F6) subtil sichtbar

Precondition Checklist:
- Aktuell: trockene Liste
- Polish: jede Precondition als Pill mit Icon, Color-Coded (Green Check / Red Alert), Tooltip mit Detail
- Inline am Engage Button, nicht separate Liste
- Preconditions: Engine running, Cruise control active, ETS2 Navigation active, Telemetry healthy, vJoy device available

Telemetry Sparklines (Speed, RPM):
- Gradient-Fill in Brand-Farben
- Min/Max Marker am Rand
- Smooth-Curves (Catmull-Rom)
- Live-Pulse Dot am rechten Rand mit Glow
- Hover zeigt Wert + Zeitpunkt

Big-Number Display fuer Speed/RPM:
- Mono-Font, gross
- Soll-Speed dezent darunter (kleiner, secondary color)
- Bei Speed-Change kurzer Glow

vJoy Bars:
- Steering: bipolar mit Mittel-Tick, Wert links/rechts vom Zentrum mit Brand-Gradient gefuellt
- Throttle/Brake: unipolar von links
- Marker bei wichtigen Werten (0%, 25%, 50%, 75%, 100%)
- Failsafe-Banner: rot mit Pulse, oben drueber, sehr klar sichtbar

Cruise Slider:
- Speed-Markers (50, 80, 100 km/h)
- Highlight-Range (Soll-Speed Bereich)
- On-Hover Tooltip mit km/h Wert
- Apply-Button daneben

Navi-Status Card (ersetzt Goal-UID Card):
- ETS2-Navigation Active Badge (gruen / amber wenn keine Route)
- Current Road Name (z.B. A100 Berlin)
- Next Maneuver: Icon (Left/Right/Straight/Exit) + Distance + Description
- Distance to Destination Total
- ETA
- Visual State wenn keine Navi aktiv: Hint Set destination in ETS2 to start

Plugin Toggle List:
- Status-Indicator (Dot links): green=laeuft, amber=disabled, red=Fehler
- Drag-Handle subtil aber sichtbar
- Version-Badge sehr dezent (mono, klein)
- Hover-State zeigt mehr Details

Logs:
- Log-Level Color-Coding: 
  - TRACE/DEBUG: dimmed
  - INFO: default
  - WARN: amber
  - ERROR: red
  - Nicht aufdringlich, eher Akzent am linken Rand
- Timestamp-Format: HH:MM:SS.mmm in mono, fixed width
- Filter-UI prominent oben: Level-Filter (Multi-Select), Search-Input, Target-Filter
- Auto-Scroll Indicator wenn pausiert

Blackboard Inspector:
- Tree-Expansion-Animation (kurz, unter 200ms)
- Sparkline-Style konsistent mit Telemetry
- Type-Badges: f32, u64, bool, string mit eigener Color je Typ
- Key-Path Breadcrumb oben in Detail-Pane
- Copy-Wert Button

PID Tuning:
- Slider-Style einheitlich mit Cruise Slider
- Live-Response Chart prominenter (groesser, mit Soll-Linie und Ist-Kurve)
- Save/Reset/Reload klar getrennt (Primary/Secondary/Destructive)

# Komponenten fuer geplante Features (Design-Grundlage mit erstellen)

Folgende Features sind auf der Roadmap, Design-System soll sie schon koennen:

Route-Map:
- Mini-Map View zeigt die ETS2-Navi-Route als Polyline
- Aktuelle Position als Truck-Icon, Heading-Indicator
- Naechster Maneuver-Punkt highlighted
- Geparste Map-Nodes als kleine Punkte unter der Route (Debug-Layer toggleable)
- Zoom-In/Out, Follow-Mode Toggle
- SVG oder Canvas, je nach Komplexitaet
- Style passt zum Rest, NICHT wie Google Maps

Diag/Override Mode (versteckt, nur fuer Tests):
- Tab oder Settings-Section
- Manual Goal Set via UID (Hex/Dec Input)
- Used fuer Testing wenn ETS2 nicht laeuft oder Routing isoliert getestet werden soll
- Klar gekennzeichnet als Diagnostic Mode

Speed-Limit Display:
- Aktuelles Limit als Rundes Verkehrsschild-Style Element
- Distance to next limit change
- Speed-Soll wird relativ zum Limit gesetzt (Limit minus 5 km/h etc.)

Lane-Detection Visualisierung:
- Live-Camera-Feed (klein, optional)
- Detected Lanes als ueberlagerte Polylines
- Vehicle-Heading-Indicator
- Steering-Output als Linie

Sign Detection:
- Letzte erkannte Signs als Karten-Liste
- Mit Confidence-Score und Distance
- Click zoomt in Detail mit Crop des Frames

Diagnostic Panel:
- Plugin-Latencies als horizontale Bar-Chart
- Watchdog-State Timeline
- IPC-Throughput Sparkline
- Frame-Times wenn Vision aktiv

ETS2-Status Indicator:
- Connected/Disconnected
- DLL-Version
- Last-Frame-Time
- Navigation Active flag
- Klein, im Footer oder oben rechts

Notifications/Toasts:
- Sonner ist installiert
- Style anpassen an Brand
- Severity klar: Info (Cyan), Success (Teal), Warning (Amber), Error (Red)
- Engage-Events generieren Toasts (Engaged, Disengaged, Fault)
- Navi-Events: New route detected, Destination reached, Navigation lost

# Domain-Kontext fuer Inspiration

Inspiration eher:
- Tesla Autopilot UI (klar, gross, status auf einen Blick)
- Flight Sim Cockpit-Panels (Information-Density wenn man sie braucht)
- Bloomberg Terminal Mono-Telemetry (Werte schnell lesbar)
- Game-HUDs (state-changes mit feedback)
- Linear App (Polish, dunkle Themes, Geometrie)
- Vercel Dashboard (Mono-Typography, klare Hierarchie)

Nicht: generic Admin-Dashboard, SaaS-Marketing-Site, Material-Design-by-the-book, Bootstrap-Look, fluffige Rundungen.

# Constraints

- Tauri + React + TypeScript bleibt
- shadcn als Basis OK aber darf erweitert/ersetzt werden wo es nicht passt
- Tailwind 4 ist installiert (v4 syntax)
- Lucide-react Icons OK
- Recharts fuer Charts OK, kann durch Custom-SVG ersetzt werden wo besser
- Keine neuen schweren Dependencies ohne Begruendung
- Animationen erwuenscht, abschaltbar via prefers-reduced-motion
- Dark Theme Default, Light Theme gleichwertig
- Performance: UI laeuft 24/7 waehrend Truck faehrt, keine FPS-Drops in ETS2
- Logo nutzen, Brand-Gradient als wiederkehrendes Motiv

# Aktueller Stand

Stack:
- Tauri 2 Desktop App
- React 19, TypeScript 5.7
- shadcn/ui Components (radix-basiert)
- Tailwind 4
- Zustand fuer State
- Recharts fuer Charts
- Sonner fuer Toasts
- Lucide-react Icons

Pages:
- Dashboard (Autopilot, Telemetry, Navi-Status, Cruise, vJoy Monitor)
- Plugins (List mit Toggles, Detail-View)
- Blackboard (Tree + Detail)
- Logs (Virtualized List + Filter)
- PID Tuning (Tabs steering/acc/lane_keeper + Slider + Live Response)
- Mods (Mod-Liste + Apply)
- Settings (Theme, Connection, Hotkeys, Paths, Daemon Card)

Pages haben Footer mit Connection-Status und Version.
Sidebar links, Pop-out Dashboard Button oben rechts.

# Deliverables

1. Design-System Doc (Markdown):
   - Color Tokens (CSS Custom Properties Vorschlag) fuer Dark + Light
   - Typography Tokens
   - Spacing Tokens
   - Component-States Tabelle
   - Animation-Tokens (durations, easings)

2. Layout-Vorschlaege (HTML-Mockups oder detaillierte Markdown-Wireframes):
   - Dashboard neu gedacht
   - Plugins Detail-View
   - Blackboard Inspector
   - Logs mit Filter
   - Settings

3. Komponenten-Galerie (HTML-Mock oder React-Code):
   - Engage Button alle States
   - Precondition Pills
   - Telemetry Sparkline
   - Big-Number Display
   - vJoy Bars (Steering/Throttle/Brake)
   - Failsafe Banner
   - Cruise Slider
   - Navi-Status Card (alle States: no-navi, active, near-maneuver)
   - Plugin Toggle Item
   - Log-Line
   - Blackboard Tree-Item + Detail
   - Toast (4 Severities)

4. Brand-Tokens als CSS-File oder Tailwind-Config-Snippet, direkt einsetzbar

5. Implementation-Guide:
   - Welche shadcn Components ersetzen, welche behalten
   - Wo Custom-Components noetig sind
   - Migration-Reihenfolge (was zuerst, was kann warten)
   - Geschaetzter Aufwand pro Page/Komponente

# Output-Format

Alles in einem Markdown-Doc mit eingebetteten HTML/CSS/React-Snippets wo sinnvoll. Bei den Mockups direkt zeigen statt nur beschreiben. Bei Tokens konkrete Werte, keine Platzhalter.

# Start

Beginne mit Design-System Doc (Tokens), darauf bauen die Mockups und Komponenten auf. Brand-Gradient aus dem Logo ist Anker, alles andere leitet sich daraus ab.
</pasted_text>


<system-reminder>Auto-injected reminder (ignore if not relevant): do not recreate copyrighted or branded UI unless the user's email domain matches that company. Create original designs instead.</system-reminder>

## Assistant

_[tool: update_todos]_

_[tool: copy_files]_

_[tool: write_file]_

I'll build a comprehensive design polish doc as a single scrollable HTML showcase: brand → tokens → components → page mockups → implementation guide. I'll split CSS into separate token/system files so editing stays clean.

Design direction I'm committing to:
- **Brand gradient** lifted from the logo: emerald → teal → cyan → blue → violet. Used sparingly as accent on engaged-states, sparkline glows, and primary CTAs only.
- **Dark-first** with a real (not inverted) light theme.
- **Geometric, sharp surfaces** — 6px and 10px radii max, 1px hairline borders, no fluffy shadows in dark.
- **Geist + Geist Mono** typography (modern, geometric, plays well with telemetry).
- **Tesla/Bloomberg/Linear vibes** as influence only — designs are original.

Let me build it.

_[tool: write_file]_

_[tool: write_file]_

Splitting into smaller files. Continuing.

_[tool: write_file]_

_[tool: write_file]_

Now the showcase doc. I'll split into 2 files: a layout/showcase chrome HTML, plus a JSX file for the larger interactive page mockups.

_[tool: write_file]_

Strategy: build page mockups as separate full HTML files (each linkable & poppable), then a master showcase doc that documents tokens/components and embeds the mockups in iframes.

Starting with the dashboard mockup (most important):

_[tool: write_file]_

Let me verify the dashboard renders before building more pages.

_[tool: show_html]_

_[tool: write_file]_

Continuing — I'll build the remaining mockups first, then verify everything at the end.

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

Now the main showcase doc with all the tokens, gallery, mockup previews, and implementation guide.

_[tool: update_todos]_

_[tool: done]_

_[tool: fork_verifier_agent]_

Built a complete design polish package for TruckPilot:

**Single showcase doc** (`TruckPilot Design.html`) with everything in one scroll — brand → color → type → spacing → motion → component gallery → embedded page mockups → implementation guide. Dark/Light toggle top-right.

**System direction**
- Brand gradient (mint → teal → cyan → blue → violet) sampled from the logo, used only on engaged states, sparklines, primary CTAs, route polyline, active nav rail. Never as wallpaper.
- Geometric: 6/10px radii max, 1px hairlines, no fluffy shadows in dark.
- Geist + Geist Mono — mono is mandatory for telemetry/coords/logs.
- 5 dark surfaces, 4 light, 4 semantic, 3 fg tiers — all as CSS custom properties.

**Six page mockups** — each a real HTML file, linked + iframed in the doc:
`mockups/dashboard.html`, `plugins.html`, `blackboard.html`, `logs.html`, `pid.html`, `settings.html`

**Components built**
Engage button (6 states incl. fault pulse), Precondition pills (ok/bad/warn/info), Big-number + flash animation, Sparkline with brand gradient + live pulse dot, vJoy bars (bipolar steer + unipolar throttle/brake) + Failsafe banner, Cruise slider with highlight range + ticks, Navi-status card (no-route / active / near-maneuver), Speed-limit sign + recent-signs, Plugin row (drag + status dot + toggle), Log line (4 levels + left-rail color), Blackboard tree + type badges + detail with sparkline, 4 toast severities, Route mini-map (route + maneuver + node-graph debug layer), Lane-detection overlay, Diagnostics latency bars.

**Implementation guide** — shadcn keep/replace decisions, 11 custom components with effort sizing, 4-phase migration plan (~22 dev-days), and a Tailwind v4 `@theme` drop-in.

**Notes / next steps**
- Diagnostic-override mode is on the Settings page, clearly labelled — toggle controls visibility in the real app.
- Speaker decision pending: emoji policy — currently zero; lucide-react covers everything. Confirm.
- The Route-Map is a static mock; the real one needs the parsed `.scs` node graph wired in — sized at L (5d) in the guide.

## User

TruckPilot UI Design Polish - Tauri + React Implementation

# Wichtig vorab: Output-Format

Das ist KEIN HTML-Mockup-Auftrag. TruckPilot ist eine echte Anwendung die bereits existiert und laeuft:

- Tauri 2 Desktop App (Rust-Backend, React-Frontend)
- React 19 + TypeScript 5.7
- shadcn/ui Components (Radix-basiert)
- Tailwind 4
- Zustand fuer State Management
- Recharts fuer Charts
- Sonner fuer Toasts
- Lucide-react Icons
- react-window fuer Virtualization
- react-router-dom v6

Deliverable muss als PRODUKTIONS-CODE umsetzbar sein, nicht als statische HTML-Demo. Konkret:

1. **Design-System als CSS Custom Properties + Tailwind 4 Config** - nicht inline-style Demo-HTML
2. **Komponenten als React + TypeScript Code** - mit shadcn-kompatibler API wo es passt
3. **Implementation-Anleitung Schritt fuer Schritt** - welche Datei wo aendern, welche shadcn-Component ersetzen, welche behalten, Migration-Reihenfolge
4. **Code-Snippets sind Tailwind-Klassen + React** - kein eigenstaendiges CSS, keine standalone HTML-Files

Wenn HTML-Mockups dabei sind, dann nur als visuelle Spec, IMMER mit dem entsprechenden React + Tailwind Code daneben fuer die echte Implementation.

# Rolle

Du bist Senior Product Designer + Frontend Engineer. Du designst nicht nur, du lieferst direkt umsetzbaren Code fuer eine existierende Tauri + React + shadcn App.

# Aufgabe

Look and Feel von TruckPilot komplett ueberarbeiten. TruckPilot ist ein Self-Driving Tool fuer Euro Truck Simulator 2: faehrt einen Truck autonom, zeigt Telemetry, Plugin-Status, vJoy-Output, Route-Daten und Logs in Echtzeit.

Funktional ist die UI fertig (Dashboard, Plugins, Blackboard, Logs, PID Tuning, Mods, Settings). Optisch wirkt sie wie ein generisches Admin-Dashboard. Sie soll wie ein hochwertiges Telemetry/Cockpit-Tool aussehen, das Vertrauen ausstrahlt waehrend man dem Programm die Kontrolle ueber einen 40-Tonner gibt.

# Routing-Architektur (wichtig fuer UI-Entscheidungen)

TruckPilot setzt KEINE eigene Route. Der User setzt das Ziel in der ETS2-In-Game-Navigation. TruckPilot folgt der ETS2-Navi und kombiniert das mit geparsten Map-Nodes fuer Lane-Following und Junction-Decisions.

UI-Konsequenzen:
- KEIN Goal-UID Input als primary UI (kann als Diagnostic-Mode versteckt werden)
- Primary: Navi-Status anzeigen (next maneuver, distance, current road, ETA)
- Route-Visualisierung zeigt was ETS2-Navi vorgibt
- Engage Button braucht Precondition ETS2 Navigation active

# Logo als Design-Grundlage

Das Logo ist eine Lenkrad-Silhouette mit eingearbeitetem T, geometrisch konstruiert (Kreis + Dreieck-Anschnitt). Gradient laeuft von Teal/Mint Gruen ueber Cyan/Blue zu Purple/Violet, auf dunklem Hintergrund.

Daraus ableiten:
- Brand-Gradient: Teal -> Cyan -> Blue -> Purple. Nutzbar als Accent fuer Engaged-State, Hero-Buttons, Sparkline-Gradienten.
- Geometrisch klare Schnitte, keine fluffigen Rundungen
- Dark-First, aber Light Theme bewusst gestaltet

# Scope

## 1. Visual Identity

CSS Custom Properties + Tailwind 4 Theme-Config liefern fuer:

- Farbsystem (Primary/Secondary/Accent + Semantic + 4-5 Surface-Layers Dark + Light)
- Typography (Display/Heading/Body/Mono mit konkreten Font-Empfehlungen die per Google Fonts oder Geist installierbar sind)
- Spacing-Scale
- Border-Radius, Border-Style
- Shadow-Tokens
- Animation-Tokens (durations, easings)

Format: 
- Eine `globals.css` Section mit `:root` und `[data-theme=light]` Custom Properties
- Eine Tailwind 4 Config-Section (`@theme inline { ... }` Syntax fuer v4)
- Konkrete Hex/HSL-Werte, keine Platzhalter

## 2. Layout / Information Architecture

Dashboard ist wichtigste Page. Hierarchie schaerfen:

- Primary: Engage Button (Hero)
- Status: Preconditions, Active/Engaged State, ETS2 Navigation Active
- Live-Daten: Speed, RPM, Navi-Status, vJoy-Output
- Sekundaer: Konfiguration in Tabs/Sections

Layout-Vorschlag fuer:
- Dashboard (Hero/Live/Detail Zonen)
- Plugins Detail-View
- Blackboard Inspector
- Logs mit Filter
- Settings

Format: React-JSX-Code (mit Tailwind-Klassen) fuer die Container/Grid-Struktur jeder Page, nicht standalone HTML.

Sidebar Gruppierung:
- Operation: Dashboard, Blackboard, Logs
- Configuration: Plugins, PID Tuning, Mods, Settings

## 3. Komponenten-Polish

Alle bestehenden Components als React-Code refactoren:

Engage Button (Hero):
- States: Off-ready / Off-disabled / Engaging / Engaged / Disengaging / Fault
- Brand-Gradient bei Engaged mit Pulse-Animation
- Hotkey-Indicator (F5/F6)
- Format: vollstaendige React-Component mit Props-Interface

Precondition Pills:
- Engine running, Cruise control active, ETS2 Navigation active, Telemetry healthy, vJoy device available
- Icon + Color-Coded + Tooltip
- Inline am Engage Button

Telemetry Sparklines (Speed, RPM):
- Recharts-basiert oder Custom-SVG
- Gradient-Fill in Brand-Farben
- Live-Pulse Dot rechts
- Format: React-Component mit data prop

Big-Number Display (Speed, RPM):
- Mono-Font, gross
- Value-Change Glow-Animation
- Soll-Wert dezent darunter

vJoy Bars:
- Steering bipolar, Throttle/Brake unipolar
- Brand-Gradient Fills
- Failsafe-Banner rot mit Pulse

Cruise Slider:
- Speed-Markers (50/80/100)
- On-Hover Tooltip
- shadcn Slider erweitert

Navi-Status Card (NEU, ersetzt Goal-UID Card):
- States: no-navi / active / approaching-maneuver
- Current Road Name
- Next Maneuver mit Direction-Icon + Distance
- Total Distance to Destination + ETA
- Hint "Set destination in ETS2" wenn keine Navi

Plugin Toggle Item:
- Status-Dot (laeuft/Fehler/disabled)
- Drag-Handle subtil
- Version-Badge dezent

Log Line:
- Level Color-Coding (linker Akzent-Bar)
- Mono Timestamp
- Filter-Bar oben

Blackboard Tree-Item + Detail:
- Type-Badges
- Sparkline fuer numerische Werte
- Copy-Button

Toast (Sonner customized):
- 4 Severities mit Brand-Colors
- Engage/Navi-Events

## 4. Geplante Features (Design-Grundlage)

Folgende Components designen damit sie bei Implementation bereit sind:

- Route-Map (SVG-basiert, ETS2-Navi-Route als Polyline, Map-Nodes als Debug-Layer)
- Speed-Limit Display (Verkehrsschild-Style Element)
- Lane-Detection Visualisierung (Camera-Feed + Polylines Overlay)
- Sign Detection Card-Liste
- Diagnostic Panel (Plugin-Latencies, Watchdog-Timeline)
- ETS2-Status Indicator (Footer)

Jeweils als React-Component-Skelett mit Props-Interface und Tailwind-Styling.

# Domain-Inspiration

- Tesla Autopilot UI
- Flight Sim Cockpit-Panels
- Bloomberg Terminal
- Linear App
- Vercel Dashboard

Nicht: Material-Design, Bootstrap, fluffige Rundungen, generic Admin.

# Constraints

- Stack bleibt: Tauri + React 19 + TS 5.7 + shadcn + Tailwind 4 + Zustand + Recharts + Sonner + Lucide
- shadcn-Components erweitern wo moeglich (eigene Variants), ersetzen nur wenn unvermeidbar
- Keine schweren neuen Dependencies ohne Begruendung
- Animations via Tailwind + CSS oder framer-motion (falls schon installiert, sonst CSS bevorzugen)
- prefers-reduced-motion respektieren
- Dark Default, Light gleichwertig
- Performance: UI laeuft 24/7 waehrend ETS2 laeuft

# Existing UI Stand

Pages:
- Dashboard (Autopilot, Telemetry, Route, Cruise, vJoy Monitor)
- Plugins (Liste + Toggles + Detail)
- Blackboard (Tree + Detail)
- Logs (Virtualized + Filter)
- PID Tuning (Tabs + Slider + Live Response Chart)
- Mods (Liste + Apply)
- Settings (Theme, Connection, Hotkeys, Paths, Daemon Card, IPC)

Footer: Connection-Status + Version
Sidebar links, Pop-out Dashboard Button oben rechts (F2)

# Deliverables (verbindlich)

## A) Design-System Tokens

Datei: `design-system/tokens.md`

- CSS Custom Properties fuer Light + Dark (in `globals.css` einfuegbar)
- Tailwind 4 `@theme inline {}` Block (in `globals.css` direkt nach den Custom Properties)
- Konkrete Werte (Hex/HSL), keine Platzhalter
- Code-Block direkt copy-paste-fertig

## B) Implementation-Anleitung Schritt fuer Schritt

Datei: `design-system/implementation-guide.md`

Pro Phase eine Sektion:

**Phase 1: Design-Tokens etablieren**
- Datei: `crates/ui/src/styles/globals.css` (oder wo die globalen Styles liegen)
- Was austauschen / hinzufuegen
- Was an `tailwind.config.ts` aendern (falls noch nicht v4-Inline)
- Wie testen (Theme-Switch in Settings sollte sofort durchziehen)

**Phase 2: Sidebar + Navigation neu**
- Datei: vermutlich `crates/ui/src/components/Sidebar.tsx` oder `Layout.tsx`
- Konkreter Diff: was raus, was rein
- Gruppierung Operation/Configuration

**Phase 3: Dashboard Layout**
- Datei: `crates/ui/src/routes/Dashboard.tsx` (oder Ueberseite)
- Hero/Live/Detail Zonen
- Welche existing Cards ersetzen, welche umsortieren

**Phase 4: Hero-Components (Engage Button + Preconditions + Navi-Status)**
- Pro Component: Datei-Pfad, neue Files erstellen, Props-Interface, JSX
- Wie an Blackboard anschliessen (Zustand-Selektoren existieren bereits)

**Phase 5: Live-Display Components (Telemetry Sparkline, Big-Number, vJoy Bars)**
- Refactoring der existing Components
- Wo Recharts erweitert / Custom-SVG noetig

**Phase 6: Page-by-Page Polish (Plugins, Blackboard, Logs, PID, Mods, Settings)**
- Pro Page: Datei-Pfad, was austauschen
- Konsistenz mit Tokens sicherstellen

**Phase 7: Toasts + Notifications**
- Sonner-Customization
- Welche Events bekommen Toasts

**Phase 8: Animations + prefers-reduced-motion**
- CSS-Keyframes oder Tailwind-Utilities
- Test wie es sich anfuehlt

Pro Phase: ungefaehrer Aufwand, Risiken, Akzeptanzkriterium.

## C) Komponenten-Code

Datei: `design-system/components/` als Mini-Library
Pro Component eine .tsx-Datei mit:
- Props-Interface
- Implementation mit Tailwind-Klassen + shadcn-Primitives wo passend
- Default-Export
- Beispiel-Verwendung als Comment oben

Mindestens:
- EngageButton.tsx
- PreconditionPill.tsx
- TelemetrySparkline.tsx
- BigNumberDisplay.tsx
- VJoyBar.tsx (mit Variants: bipolar/unipolar)
- FailsafeBanner.tsx
- CruiseSlider.tsx
- NaviStatusCard.tsx
- PluginToggleItem.tsx
- LogLine.tsx
- BlackboardTreeItem.tsx
- BlackboardDetail.tsx
- CustomToast.tsx (Sonner-Wrapper)
- RouteMap.tsx (Skelett mit SVG-Struktur)
- SpeedLimitDisplay.tsx
- DiagnosticPanel.tsx (Skelett)
- ETS2StatusIndicator.tsx

Die Components sollen so geschrieben sein dass sie 1:1 in `crates/ui/src/components/` kopierbar sind (mit Anpassung der Imports an existing structure).

## D) Visuelle Spec (optional, sekundaer)

Wenn HTML-Mockups helfen die Vision zu kommunizieren, dann als zusaetzliche Datei `design-system/visual-spec.html`. Aber primaer ist Code, nicht Mockup.

# Nicht-Ziele

- Standalone HTML/CSS-Demo die nicht in die React-App passt
- Tailwind v3 Syntax (wir nutzen v4)
- Neue UI-Frameworks vorschlagen (kein Mantine, kein Chakra, kein Material)
- Neue State-Libraries (Zustand bleibt)
- Eigene Build-Tools

# Format-Hinweise

- Code in Markdown-Fences mit Sprach-Tag (```tsx, ```css, ```ts)
- Datei-Pfade als Comments oder Headers (e.g. `// crates/ui/src/components/EngageButton.tsx`)
- Bei Component-Beispielen: Imports vollstaendig zeigen
- Klassen-Namen Tailwind, kein inline `style={{...}}` ausser fuer dynamische Werte

# Start

Beginne mit Deliverable A (Design-System Tokens). Konkrete CSS-Custom-Properties + Tailwind 4 Config. Erst wenn die Tokens stehen, weiter zu B (Implementation-Anleitung) und C (Komponenten-Code). Brand-Gradient aus dem Logo ist Anker, alles andere leitet sich daraus ab.

Ausgabe alles in einer langen Antwort (oder strukturiert in benannte Files falls dein UI das unterstuetzt). Wichtig: zuerst Tokens, dann Anleitung, dann Components, am Schluss optional die visuelle Spec.

## User

<system-info comment="Only acknowledge these if relevant">
Project title is now "TruckPilot UI"
Current date is now May 18, 2026
</system-info>

<pasted_text name="Pasted text (313 lines)">
TruckPilot UI Design Polish - Tauri + React Implementation

# Wichtig vorab: Output-Format

Das ist KEIN HTML-Mockup-Auftrag. TruckPilot ist eine echte Anwendung die bereits existiert und laeuft:

- Tauri 2 Desktop App (Rust-Backend, React-Frontend)
- React 19 + TypeScript 5.7
- shadcn/ui Components (Radix-basiert)
- Tailwind 4
- Zustand fuer State Management
- Recharts fuer Charts
- Sonner fuer Toasts
- Lucide-react Icons
- react-window fuer Virtualization
- react-router-dom v6

Deliverable muss als PRODUKTIONS-CODE umsetzbar sein, nicht als statische HTML-Demo. Konkret:

1. **Design-System als CSS Custom Properties + Tailwind 4 Config** - nicht inline-style Demo-HTML
2. **Komponenten als React + TypeScript Code** - mit shadcn-kompatibler API wo es passt
3. **Implementation-Anleitung Schritt fuer Schritt** - welche Datei wo aendern, welche shadcn-Component ersetzen, welche behalten, Migration-Reihenfolge
4. **Code-Snippets sind Tailwind-Klassen + React** - kein eigenstaendiges CSS, keine standalone HTML-Files

Wenn HTML-Mockups dabei sind, dann nur als visuelle Spec, IMMER mit dem entsprechenden React + Tailwind Code daneben fuer die echte Implementation.

# Rolle

Du bist Senior Product Designer + Frontend Engineer. Du designst nicht nur, du lieferst direkt umsetzbaren Code fuer eine existierende Tauri + React + shadcn App.

# Aufgabe

Look and Feel von TruckPilot komplett ueberarbeiten. TruckPilot ist ein Self-Driving Tool fuer Euro Truck Simulator 2: faehrt einen Truck autonom, zeigt Telemetry, Plugin-Status, vJoy-Output, Route-Daten und Logs in Echtzeit.

Funktional ist die UI fertig (Dashboard, Plugins, Blackboard, Logs, PID Tuning, Mods, Settings). Optisch wirkt sie wie ein generisches Admin-Dashboard. Sie soll wie ein hochwertiges Telemetry/Cockpit-Tool aussehen, das Vertrauen ausstrahlt waehrend man dem Programm die Kontrolle ueber einen 40-Tonner gibt.

# Routing-Architektur (wichtig fuer UI-Entscheidungen)

TruckPilot setzt KEINE eigene Route. Der User setzt das Ziel in der ETS2-In-Game-Navigation. TruckPilot folgt der ETS2-Navi und kombiniert das mit geparsten Map-Nodes fuer Lane-Following und Junction-Decisions.

UI-Konsequenzen:
- KEIN Goal-UID Input als primary UI (kann als Diagnostic-Mode versteckt werden)
- Primary: Navi-Status anzeigen (next maneuver, distance, current road, ETA)
- Route-Visualisierung zeigt was ETS2-Navi vorgibt
- Engage Button braucht Precondition ETS2 Navigation active

# Logo als Design-Grundlage

Das Logo ist eine Lenkrad-Silhouette mit eingearbeitetem T, geometrisch konstruiert (Kreis + Dreieck-Anschnitt). Gradient laeuft von Teal/Mint Gruen ueber Cyan/Blue zu Purple/Violet, auf dunklem Hintergrund.

Daraus ableiten:
- Brand-Gradient: Teal -> Cyan -> Blue -> Purple. Nutzbar als Accent fuer Engaged-State, Hero-Buttons, Sparkline-Gradienten.
- Geometrisch klare Schnitte, keine fluffigen Rundungen
- Dark-First, aber Light Theme bewusst gestaltet

# Scope

## 1. Visual Identity

CSS Custom Properties + Tailwind 4 Theme-Config liefern fuer:

- Farbsystem (Primary/Secondary/Accent + Semantic + 4-5 Surface-Layers Dark + Light)
- Typography (Display/Heading/Body/Mono mit konkreten Font-Empfehlungen die per Google Fonts oder Geist installierbar sind)
- Spacing-Scale
- Border-Radius, Border-Style
- Shadow-Tokens
- Animation-Tokens (durations, easings)

Format: 
- Eine `globals.css` Section mit `:root` und `[data-theme=light]` Custom Properties
- Eine Tailwind 4 Config-Section (`@theme inline { ... }` Syntax fuer v4)
- Konkrete Hex/HSL-Werte, keine Platzhalter

## 2. Layout / Information Architecture

Dashboard ist wichtigste Page. Hierarchie schaerfen:

- Primary: Engage Button (Hero)
- Status: Preconditions, Active/Engaged State, ETS2 Navigation Active
- Live-Daten: Speed, RPM, Navi-Status, vJoy-Output
- Sekundaer: Konfiguration in Tabs/Sections

Layout-Vorschlag fuer:
- Dashboard (Hero/Live/Detail Zonen)
- Plugins Detail-View
- Blackboard Inspector
- Logs mit Filter
- Settings

Format: React-JSX-Code (mit Tailwind-Klassen) fuer die Container/Grid-Struktur jeder Page, nicht standalone HTML.

Sidebar Gruppierung:
- Operation: Dashboard, Blackboard, Logs
- Configuration: Plugins, PID Tuning, Mods, Settings

## 3. Komponenten-Polish

Alle bestehenden Components als React-Code refactoren:

Engage Button (Hero):
- States: Off-ready / Off-disabled / Engaging / Engaged / Disengaging / Fault
- Brand-Gradient bei Engaged mit Pulse-Animation
- Hotkey-Indicator (F5/F6)
- Format: vollstaendige React-Component mit Props-Interface

Precondition Pills:
- Engine running, Cruise control active, ETS2 Navigation active, Telemetry healthy, vJoy device available
- Icon + Color-Coded + Tooltip
- Inline am Engage Button

Telemetry Sparklines (Speed, RPM):
- Recharts-basiert oder Custom-SVG
- Gradient-Fill in Brand-Farben
- Live-Pulse Dot rechts
- Format: React-Component mit data prop

Big-Number Display (Speed, RPM):
- Mono-Font, gross
- Value-Change Glow-Animation
- Soll-Wert dezent darunter

vJoy Bars:
- Steering bipolar, Throttle/Brake unipolar
- Brand-Gradient Fills
- Failsafe-Banner rot mit Pulse

Cruise Slider:
- Speed-Markers (50/80/100)
- On-Hover Tooltip
- shadcn Slider erweitert

Navi-Status Card (NEU, ersetzt Goal-UID Card):
- States: no-navi / active / approaching-maneuver
- Current Road Name
- Next Maneuver mit Direction-Icon + Distance
- Total Distance to Destination + ETA
- Hint "Set destination in ETS2" wenn keine Navi

Plugin Toggle Item:
- Status-Dot (laeuft/Fehler/disabled)
- Drag-Handle subtil
- Version-Badge dezent

Log Line:
- Level Color-Coding (linker Akzent-Bar)
- Mono Timestamp
- Filter-Bar oben

Blackboard Tree-Item + Detail:
- Type-Badges
- Sparkline fuer numerische Werte
- Copy-Button

Toast (Sonner customized):
- 4 Severities mit Brand-Colors
- Engage/Navi-Events

## 4. Geplante Features (Design-Grundlage)

Folgende Components designen damit sie bei Implementation bereit sind:

- Route-Map (SVG-basiert, ETS2-Navi-Route als Polyline, Map-Nodes als Debug-Layer)
- Speed-Limit Display (Verkehrsschild-Style Element)
- Lane-Detection Visualisierung (Camera-Feed + Polylines Overlay)
- Sign Detection Card-Liste
- Diagnostic Panel (Plugin-Latencies, Watchdog-Timeline)
- ETS2-Status Indicator (Footer)

Jeweils als React-Component-Skelett mit Props-Interface und Tailwind-Styling.

# Domain-Inspiration

- Tesla Autopilot UI
- Flight Sim Cockpit-Panels
- Bloomberg Terminal
- Linear App
- Vercel Dashboard

Nicht: Material-Design, Bootstrap, fluffige Rundungen, generic Admin.

# Constraints

- Stack bleibt: Tauri + React 19 + TS 5.7 + shadcn + Tailwind 4 + Zustand + Recharts + Sonner + Lucide
- shadcn-Components erweitern wo moeglich (eigene Variants), ersetzen nur wenn unvermeidbar
- Keine schweren neuen Dependencies ohne Begruendung
- Animations via Tailwind + CSS oder framer-motion (falls schon installiert, sonst CSS bevorzugen)
- prefers-reduced-motion respektieren
- Dark Default, Light gleichwertig
- Performance: UI laeuft 24/7 waehrend ETS2 laeuft

# Existing UI Stand

Pages:
- Dashboard (Autopilot, Telemetry, Route, Cruise, vJoy Monitor)
- Plugins (Liste + Toggles + Detail)
- Blackboard (Tree + Detail)
- Logs (Virtualized + Filter)
- PID Tuning (Tabs + Slider + Live Response Chart)
- Mods (Liste + Apply)
- Settings (Theme, Connection, Hotkeys, Paths, Daemon Card, IPC)

Footer: Connection-Status + Version
Sidebar links, Pop-out Dashboard Button oben rechts (F2)

# Deliverables (verbindlich)

## A) Design-System Tokens

Datei: `design-system/tokens.md`

- CSS Custom Properties fuer Light + Dark (in `globals.css` einfuegbar)
- Tailwind 4 `@theme inline {}` Block (in `globals.css` direkt nach den Custom Properties)
- Konkrete Werte (Hex/HSL), keine Platzhalter
- Code-Block direkt copy-paste-fertig

## B) Implementation-Anleitung Schritt fuer Schritt

Datei: `design-system/implementation-guide.md`

Pro Phase eine Sektion:

**Phase 1: Design-Tokens etablieren**
- Datei: `crates/ui/src/styles/globals.css` (oder wo die globalen Styles liegen)
- Was austauschen / hinzufuegen
- Was an `tailwind.config.ts` aendern (falls noch nicht v4-Inline)
- Wie testen (Theme-Switch in Settings sollte sofort durchziehen)

**Phase 2: Sidebar + Navigation neu**
- Datei: vermutlich `crates/ui/src/components/Sidebar.tsx` oder `Layout.tsx`
- Konkreter Diff: was raus, was rein
- Gruppierung Operation/Configuration

**Phase 3: Dashboard Layout**
- Datei: `crates/ui/src/routes/Dashboard.tsx` (oder Ueberseite)
- Hero/Live/Detail Zonen
- Welche existing Cards ersetzen, welche umsortieren

**Phase 4: Hero-Components (Engage Button + Preconditions + Navi-Status)**
- Pro Component: Datei-Pfad, neue Files erstellen, Props-Interface, JSX
- Wie an Blackboard anschliessen (Zustand-Selektoren existieren bereits)

**Phase 5: Live-Display Components (Telemetry Sparkline, Big-Number, vJoy Bars)**
- Refactoring der existing Components
- Wo Recharts erweitert / Custom-SVG noetig

**Phase 6: Page-by-Page Polish (Plugins, Blackboard, Logs, PID, Mods, Settings)**
- Pro Page: Datei-Pfad, was austauschen
- Konsistenz mit Tokens sicherstellen

**Phase 7: Toasts + Notifications**
- Sonner-Customization
- Welche Events bekommen Toasts

**Phase 8: Animations + prefers-reduced-motion**
- CSS-Keyframes oder Tailwind-Utilities
- Test wie es sich anfuehlt

Pro Phase: ungefaehrer Aufwand, Risiken, Akzeptanzkriterium.

## C) Komponenten-Code

Datei: `design-system/components/` als Mini-Library
Pro Component eine .tsx-Datei mit:
- Props-Interface
- Implementation mit Tailwind-Klassen + shadcn-Primitives wo passend
- Default-Export
- Beispiel-Verwendung als Comment oben

Mindestens:
- EngageButton.tsx
- PreconditionPill.tsx
- TelemetrySparkline.tsx
- BigNumberDisplay.tsx
- VJoyBar.tsx (mit Variants: bipolar/unipolar)
- FailsafeBanner.tsx
- CruiseSlider.tsx
- NaviStatusCard.tsx
- PluginToggleItem.tsx
- LogLine.tsx
- BlackboardTreeItem.tsx
- BlackboardDetail.tsx
- CustomToast.tsx (Sonner-Wrapper)
- RouteMap.tsx (Skelett mit SVG-Struktur)
- SpeedLimitDisplay.tsx
- DiagnosticPanel.tsx (Skelett)
- ETS2StatusIndicator.tsx

Die Components sollen so geschrieben sein dass sie 1:1 in `crates/ui/src/components/` kopierbar sind (mit Anpassung der Imports an existing structure).

## D) Visuelle Spec (optional, sekundaer)

Wenn HTML-Mockups helfen die Vision zu kommunizieren, dann als zusaetzliche Datei `design-system/visual-spec.html`. Aber primaer ist Code, nicht Mockup.

# Nicht-Ziele

- Standalone HTML/CSS-Demo die nicht in die React-App passt
- Tailwind v3 Syntax (wir nutzen v4)
- Neue UI-Frameworks vorschlagen (kein Mantine, kein Chakra, kein Material)
- Neue State-Libraries (Zustand bleibt)
- Eigene Build-Tools

# Format-Hinweise

- Code in Markdown-Fences mit Sprach-Tag (```tsx, ```css, ```ts)
- Datei-Pfade als Comments oder Headers (e.g. `// crates/ui/src/components/EngageButton.tsx`)
- Bei Component-Beispielen: Imports vollstaendig zeigen
- Klassen-Namen Tailwind, kein inline `style={{...}}` ausser fuer dynamische Werte

# Start

Beginne mit Deliverable A (Design-System Tokens). Konkrete CSS-Custom-Properties + Tailwind 4 Config. Erst wenn die Tokens stehen, weiter zu B (Implementation-Anleitung) und C (Komponenten-Code). Brand-Gradient aus dem Logo ist Anker, alles andere leitet sich daraus ab.

Ausgabe alles in einer langen Antwort (oder strukturiert in benannte Files falls dein UI das unterstuetzt). Wichtig: zuerst Tokens, dann Anleitung, dann Components, am Schluss optional die visuelle Spec.
</pasted_text>

## Assistant

## Assistant


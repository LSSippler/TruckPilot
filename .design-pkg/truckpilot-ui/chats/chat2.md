# Chat

_Started 2026-05-18 17:41 UTC_

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


<system-reminder>Auto-injected reminder (ignore if not relevant): do not recreate copyrighted or branded UI unless the user's email domain matches that company. Create original designs instead.</system-reminder>

## User

<system-info comment="Only acknowledge these if relevant">
Project title is now "TruckPilot UI"
Current date is now May 18, 2026
</system-info>

<pasted_text name="Pasted text (353 lines)">
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

1. Design-System als CSS Custom Properties + Tailwind 4 Config, nicht inline-style Demo-HTML
2. Komponenten als React + TypeScript Code, mit shadcn-kompatibler API wo es passt
3. Implementation-Anleitung Schritt fuer Schritt: welche Datei wo aendern, welche shadcn-Component ersetzen, welche behalten, Migration-Reihenfolge
4. Code-Snippets sind Tailwind-Klassen + React, kein eigenstaendiges CSS, keine standalone HTML-Files

Wenn HTML-Mockups dabei sind, dann nur als visuelle Spec, IMMER mit dem entsprechenden React + Tailwind Code daneben fuer die echte Implementation.

# Designhaltung: zurueckhaltend modern

Wichtig: das UI soll modern sein, aber nicht overdesigned. Keine Effekt-Show. Klare Hierarchie, ruhige Flaechen, wenige starke Akzente. Inspiration: Linear, Vercel, Stripe Dashboard. Nicht: Game-HUDs voller Glows, Glassmorphism-Cards, Neon-Effekte ueberall.

Konkrete Leitplanken:

- Brand-Gradient sparsam einsetzen. Nicht auf jedem Button, nicht in jeder Card. Hauptsaechlich: Engage-Button im Engaged-State, Logo, vielleicht ein Active-Sparkline.
- Animationen subtil und kurz (unter 200ms). Kein dauerhaftes Pulsieren ausser bei wirklich kritischem State (Engaged, Failsafe).
- Hover-States dezent (Border-Shift, leichter Background-Change), kein Lift mit Shadow.
- Keine Glows, keine Glassmorphism, keine Backdrop-Blurs ausser Modal-Overlay.
- Borders bevorzugt vor Shadows fuer Card-Trennung im Dark Theme.
- Mono-Font fuer Werte, Sans fuer Text, kein Display-Font-Schnickschnack.
- Farben gedeckt im Standard-State. Akzent-Farbe nur fuer eine Sache pro Screen.

Wenn etwas zwischen ruhig und auffaellig zu entscheiden ist: ruhig.

# Rolle

Du bist Senior Product Designer + Frontend Engineer. Du designst nicht nur, du lieferst direkt umsetzbaren Code fuer eine existierende Tauri + React + shadcn App. Du verstehst dass die App professional und ruhig aussehen soll, nicht impressiv-sein-wollend.

# Aufgabe

Look and Feel von TruckPilot ueberarbeiten. TruckPilot ist ein Self-Driving Tool fuer Euro Truck Simulator 2: faehrt einen Truck autonom, zeigt Telemetry, Plugin-Status, vJoy-Output, Route-Daten und Logs in Echtzeit.

Funktional ist die UI fertig (Dashboard, Plugins, Blackboard, Logs, PID Tuning, Mods, Settings). Optisch wirkt sie wie ein generisches Admin-Dashboard. Sie soll wie ein professionelles Telemetry/Cockpit-Tool aussehen, das Vertrauen ausstrahlt waehrend man dem Programm die Kontrolle ueber einen 40-Tonner gibt. Vertrauen kommt aus Klarheit und Ruhe, nicht aus visuellen Effekten.

# Routing-Architektur (wichtig fuer UI-Entscheidungen)

TruckPilot setzt KEINE eigene Route. Der User setzt das Ziel in der ETS2-In-Game-Navigation. TruckPilot folgt der ETS2-Navi und kombiniert das mit geparsten Map-Nodes fuer Lane-Following und Junction-Decisions.

UI-Konsequenzen:
- KEIN Goal-UID Input als primary UI (kann als Diagnostic-Mode versteckt werden)
- Primary: Navi-Status anzeigen (next maneuver, distance, current road, ETA)
- Route-Visualisierung zeigt was ETS2-Navi vorgibt
- Engage Button braucht Precondition ETS2 Navigation active

# Logo als Design-Grundlage

Das Logo ist eine Lenkrad-Silhouette mit eingearbeitetem T, geometrisch konstruiert (Kreis + Dreieck-Anschnitt). Gradient laeuft von Teal/Mint Gruen ueber Cyan/Blue zu Purple/Violet, auf dunklem Hintergrund.

Aus dem Logo ableiten:
- Brand-Akzentfarbe: Teal/Mint (ein einzelner Solid-Wert, nicht der ganze Gradient als Default)
- Brand-Gradient: vorhanden, aber sparsam einsetzen (Logo selbst, Engaged-State-Highlight, sehr wenige Hero-Stellen)
- Geometrisch klar, gerade Linien, kleine Radien (4-6px max), keine Pillen-Buttons ueberall

Dark-First: Logo wirkt auf Dunkel am staerksten. Light Theme bewusst gestaltet, nicht Color-Invert.

# Scope

## 1. Visual Identity

CSS Custom Properties + Tailwind 4 Theme-Config liefern fuer:

- Farbsystem
  - Primary-Akzent: ein Teal/Mint solid (z.B. emerald-Bereich), als Default-Akzent
  - Brand-Gradient: separat definiert, NUR fuer dedizierte Stellen
  - Semantic: Success / Warning / Error / Info, gedeckte Werte
  - Surfaces: maximal 4 Layer (Base, Card, Elevated, Overlay), nicht 6
  - Borders: 1-2 Subtile Werte, klare Hierarchie
  - Text: Primary, Secondary, Muted, Disabled
- Typography
  - Sans-Serif fuer alles UI (Inter oder Geist)
  - Mono fuer Werte/Logs/UIDs (JetBrains Mono oder Geist Mono)
  - 4-5 Sizes total, nicht mehr
- Spacing-Scale (4/8/12/16/24/32/48)
- Border-Radius: max 2-3 Werte (4px Default, 6px Cards, 999px nur fuer Pills/Avatars)
- Shadows: minimal nutzen, im Dark Theme fast garnicht, im Light Theme dezent
- Animation-Tokens: Duration 120/180/240ms, Easing ease-out fuer alles UI

Format: 
- Eine globals.css Section mit :root und [data-theme=light] Custom Properties
- Eine Tailwind 4 Config-Section (@theme inline { ... } Syntax fuer v4)
- Konkrete Hex/HSL-Werte, keine Platzhalter

## 2. Layout / Information Architecture

Dashboard ist wichtigste Page. Hierarchie schaerfen:

- Primary: Engage Button (Hero-Element, aber nicht riesig-protzig)
- Status: Preconditions kompakt am Engage Button
- Live-Daten: Speed, Navi-Status, vJoy-Output in einer ruhigen Anordnung
- Sekundaer: alles andere in Tabs/Sections

Layout-Vorschlag fuer:
- Dashboard
- Plugins Detail-View
- Blackboard Inspector
- Logs mit Filter
- Settings

Format: React-JSX-Code (mit Tailwind-Klassen) fuer die Container/Grid-Struktur jeder Page.

Sidebar Gruppierung:
- Operation: Dashboard, Blackboard, Logs
- Configuration: Plugins, PID Tuning, Mods, Settings

## 3. Komponenten-Polish

Alle bestehenden Components als React-Code refactoren:

Engage Button:
- States: Off-ready / Off-disabled / Engaging / Engaged / Disengaging / Fault
- Im Engaged-State: subtiler Brand-Gradient-Akzent (z.B. Border oder schmale Linie), kein vollflaechiges Gradient-Bombardement
- Hover/Focus-Ring dezent
- Hotkey-Indicator (F5/F6) als kleines Mono-Badge
- Format: vollstaendige React-Component mit Props-Interface

Precondition Pills:
- Engine running, Cruise control active, ETS2 Navigation active, Telemetry healthy, vJoy device available
- Kleines Icon + kompakter Text, kein dickes Pill-Design
- Color-Coded subtil (Status-Dot links statt Background-Fill)
- Tooltip mit Detail

Telemetry Sparklines (Speed, RPM):
- Recharts-basiert
- Single-Color-Stroke (Akzent), kein Gradient-Fill ueberall
- Optional dezenter Gradient-Fill mit niedriger Opacity unter der Linie
- Letzter Datenpunkt als kleiner Dot, kein Glow
- Hover zeigt Wert

Big-Number Display (Speed, RPM):
- Mono-Font, gross aber nicht uebertrieben
- Value-Change ohne Glow, ggf. minimaler Color-Shift fuer 100ms
- Soll-Wert dezent darunter (kleiner, muted)

vJoy Bars:
- Steering bipolar (Center-Tick), Throttle/Brake unipolar
- Solid-Akzentfarbe als Fill, kein Gradient
- Failsafe-Banner: rot Akzent, Border-Left oder kompakter Bar, kein Vollflaechen-Pulse

Cruise Slider:
- shadcn Slider als Basis, mit dezenten Tick-Markers (50/80/100)
- Tooltip bei Hover

Navi-Status Card (NEU, ersetzt Goal-UID Card):
- States: no-navi / active / approaching-maneuver
- Next Maneuver mit Direction-Icon + Distance gross und klar
- Total Distance + ETA daneben sekundaer
- Im no-navi State: ruhiger Empty-State, kein FAULT-Banner

Plugin Toggle Item:
- Status-Dot (laeuft/Fehler/disabled) klein, links
- shadcn Switch, Drag-Handle subtil als kleines Icon
- Version-Badge dezent muted

Log Line:
- Level Color-Coding als 2px Border-Left, nicht voller Background
- Mono Timestamp, fixed width
- Filter-Bar oben minimal

Blackboard Tree-Item + Detail:
- Tree mit kleinen Chevron-Icons, klare Hierarchie durch Indent
- Type-Badges klein und muted
- Sparkline fuer numerische Werte: einzelne Linie, ein Akzent

Toast (Sonner customized):
- 4 Severities mit semantischen Farben (nicht Brand-Gradient)
- Engage/Navi-Events
- Kurz, klein, keine grossen Banner

## 4. Geplante Features (Design-Grundlage)

Folgende Components designen damit sie bei Implementation bereit sind:

- Route-Map (SVG, ETS2-Navi-Route als Polyline mit Brand-Akzent, Map-Nodes als kleine muted Punkte im Debug-Layer)
- Speed-Limit Display (klein, rundes Verkehrsschild-Style Element)
- Lane-Detection Visualisierung (Camera-Feed klein, Polylines dezent overlay)
- Sign Detection Card-Liste (kompakt)
- Diagnostic Panel (Plugin-Latencies als Bar-Chart, Watchdog-Timeline minimal)
- ETS2-Status Indicator (Footer, ein Dot + Text)

Jeweils als React-Component-Skelett mit Props-Interface und Tailwind-Styling.

# Domain-Inspiration

- Linear App
- Vercel Dashboard
- Stripe Dashboard
- Apple Numbers fuer Datendichte
- Bloomberg Terminal nur fuer Mono-Werte-Layout, nicht fuer Aesthetik

Nicht: Material-Design, Bootstrap, fluffige Rundungen, generic Admin, Game-HUD, Glassmorphism, Cyberpunk-Style, Neon.

# Constraints

- Stack bleibt: Tauri + React 19 + TS 5.7 + shadcn + Tailwind 4 + Zustand + Recharts + Sonner + Lucide
- shadcn-Components erweitern wo moeglich (eigene Variants), ersetzen nur wenn unvermeidbar
- Keine schweren neuen Dependencies ohne Begruendung
- Animations via Tailwind + CSS, kein framer-motion einbringen wenn nicht schon da
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

Datei: design-system/tokens.md

- CSS Custom Properties fuer Light + Dark (in globals.css einfuegbar)
- Tailwind 4 @theme inline {} Block (in globals.css direkt nach den Custom Properties)
- Konkrete Werte (Hex/HSL), keine Platzhalter
- Code-Block direkt copy-paste-fertig

## B) Implementation-Anleitung Schritt fuer Schritt

Datei: design-system/implementation-guide.md

Pro Phase eine Sektion:

Phase 1: Design-Tokens etablieren
- Datei: crates/ui/src/styles/globals.css (oder wo die globalen Styles liegen)
- Was austauschen / hinzufuegen
- Was an tailwind.config.ts aendern (falls noch nicht v4-Inline)
- Wie testen (Theme-Switch in Settings sollte sofort durchziehen)

Phase 2: Sidebar + Navigation neu
- Datei: vermutlich crates/ui/src/components/Sidebar.tsx oder Layout.tsx
- Konkreter Diff: was raus, was rein
- Gruppierung Operation/Configuration

Phase 3: Dashboard Layout
- Datei: crates/ui/src/routes/Dashboard.tsx
- Hero/Live/Detail Zonen
- Welche existing Cards ersetzen, welche umsortieren

Phase 4: Hero-Components (Engage Button + Preconditions + Navi-Status)
- Pro Component: Datei-Pfad, neue Files erstellen, Props-Interface, JSX
- Wie an Blackboard anschliessen (Zustand-Selektoren existieren bereits)

Phase 5: Live-Display Components (Telemetry Sparkline, Big-Number, vJoy Bars)
- Refactoring der existing Components
- Wo Recharts angepasst werden muss

Phase 6: Page-by-Page Polish (Plugins, Blackboard, Logs, PID, Mods, Settings)
- Pro Page: Datei-Pfad, was austauschen
- Konsistenz mit Tokens sicherstellen

Phase 7: Toasts + Notifications
- Sonner-Customization
- Welche Events bekommen Toasts

Phase 8: Animations + prefers-reduced-motion
- CSS-Keyframes oder Tailwind-Utilities
- Test wie es sich anfuehlt, ggf. zurueckdrehen wenn zu viel

Pro Phase: ungefaehrer Aufwand, Risiken, Akzeptanzkriterium.

## C) Komponenten-Code

Datei: design-system/components/ als Mini-Library
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

Die Components sollen so geschrieben sein dass sie 1:1 in crates/ui/src/components/ kopierbar sind.

## D) Visuelle Spec (optional, sekundaer)

Wenn HTML-Mockups helfen die Vision zu kommunizieren, dann als zusaetzliche Datei design-system/visual-spec.html. Aber primaer ist Code, nicht Mockup.

# Nicht-Ziele

- Standalone HTML/CSS-Demo die nicht in die React-App passt
- Tailwind v3 Syntax (wir nutzen v4)
- Neue UI-Frameworks vorschlagen (kein Mantine, kein Chakra, kein Material)
- Neue State-Libraries (Zustand bleibt)
- Eigene Build-Tools
- Overdesign: kein Glassmorphism, keine Backdrop-Blurs ueberall, keine vollflaechigen Gradients, keine Daueranimations

# Selbst-Check fuer den Designer

Bevor du eine Komponente abschickst, frag dich:
- Brauche ich diesen Akzent wirklich, oder ist Standard-Text auch klar genug?
- Brauche ich diese Animation, oder lenkt sie ab?
- Brauche ich diesen Gradient, oder reicht ein Solid?
- Wuerde Linear/Vercel/Stripe das so machen, oder waere ihnen das zu viel?

Wenn die Antwort drei mal "reicht auch ohne" ist, dann ohne.

# Format-Hinweise

- Code in Markdown-Fences mit Sprach-Tag (```tsx, ```css, ```ts)
- Datei-Pfade als Comments oder Headers
- Bei Component-Beispielen: Imports vollstaendig zeigen
- Klassen-Namen Tailwind, kein inline style={{...}} ausser fuer dynamische Werte

# Start

Beginne mit Deliverable A (Design-System Tokens). Konkrete CSS-Custom-Properties + Tailwind 4 Config. Erst wenn die Tokens stehen, weiter zu B (Implementation-Anleitung) und C (Komponenten-Code).

Ausgabe alles in einer langen Antwort (oder strukturiert in benannte Files falls dein UI das unterstuetzt). Wichtig: zuerst Tokens, dann Anleitung, dann Components, am Schluss optional die visuelle Spec.
</pasted_text>

## Assistant

_[tool: update_todos]_

_[tool: write_file]_

_[tool: write_file]_

Now the components. I'll write each as a self-contained `.tsx`.

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: str_replace_edit]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

_[tool: write_file]_

Now an index README and we're done.

_[tool: done]_

_[tool: fork_verifier_agent]_

Geliefert in `design-system/`:

- **A) `tokens.md`** — komplette CSS Custom Properties (Dark + Light) und Tailwind 4 `@theme inline` Block, copy-paste-fertig für `globals.css`. Ein Solid-Teal als Default-Akzent, der volle Logo-Gradient explizit *nicht* als Utility (damit er nicht versehentlich überall landet).
- **B) `implementation-guide.md`** — 8 Phasen mit Datei-Pfaden, Diffs, Risiken und Akzeptanzkriterien (~3,5 Entwicklertage gesamt; jede Phase ist ein eigener PR).
- **C) `components/`** — 17 `.tsx`-Dateien, jede mit Props-Interface, Tailwind-Klassen, shadcn-Primitives wo passend, Beispiel oben als Kommentar. 1:1 nach `crates/ui/src/components/` kopierbar.
- **D) `visual-spec.html`** — statisches Dashboard-Mockup aus genau denselben Tokens (Theme-Toggle oben rechts).

Brand-Gradient erscheint an exakt **drei** Stellen: Logo, Engage-Ring im `engaged`-State, optional ein Sparkline-Fill. Pulse-Animation läuft an exakt zwei Stellen: Engage-Ring und Failsafe-Border-Left. Alles andere ist ruhig, gedeckt, mit Borders statt Shadows im Dark Theme.

Caveats / nächste Schritte:
- Imports in den `.tsx`-Files gehen von `@/lib/utils` und `@/components/ui/*` aus — Pfad-Aliases ggf. einmal anpassen.
- `RouteMap` und `DiagnosticPanel` sind Skelette mit Props-Interface, kein produktiver Code für die Daten-Anbindung.
- Tailwind v4: alte `tailwind.config.ts` löschen, falls noch da. Theme wird komplett aus CSS gelesen.

## User

<system-info comment="Only acknowledge these if relevant">
User is viewing file: design-system/visual-spec.html
</system-info>

<pasted_text name="Pasted text (280 lines)">
TruckPilot Route Preview Component — ETS2LA-Style Visualization

# Kontext

Im aktuellen Design-Output von Claude Design gibt es eine Route-Preview-Card im Dashboard. Diese soll als ETS2LA-Style Visualisierung umgesetzt werden: eigene Karte basierend auf TruckPilot's geparsten Map-Daten plus Live-Truck-Position aus Telemetry.

Das ist KEIN Screen-Capture des ETS2-Route-Advisors. Es ist eine eigenstaendige Visualisierung mit den gleichen Daten die TruckPilot eh schon hat.

# Datenquellen

TruckPilot hat folgende Daten verfuegbar:

1. graph.json (1.15M Nodes, 1.3M Edges)
   - Nodes mit Position (x, y, z)
   - Edges mit from_uid, to_uid, road-type
   - Daraus laesst sich die Strassen-Topologie zeichnen
   
2. Telemetry (live via SHM/SDK)
   - truck.position (x, y, z)
   - truck.heading (rad)
   - truck.speed
   
3. Routing-Plugin (router-plugin)
   - waypoints: Liste von Node-UIDs auf der geplanten Route
   - falls vorhanden: distance_to_next, next_maneuver_position

4. Optional: ETS2 SDK Navi
   - truck.navigation.distance (Meter bis Ziel)
   - truck.navigation.time (ETA in Sekunden)

# Component-Spec

## Layout

Card im Dashboard mit Header "ROUTE PREVIEW" plus zwei Status-Badges oben rechts:
- LIVE Badge (gruener Dot wenn Telemetry-Frames ankommen)
- Speed-Anzeige klein (aus Telemetry)

Hauptbereich: SVG-Canvas mit der Karten-Visualisierung.

Footer-Zeile mit minimalen Map-Controls:
- Zoom-Level Indikator
- Follow-Truck Toggle (Default: an)
- Debug-Layer Toggle (Default: aus, zeigt Node-Punkte)

## Visualisierung

### Was gezeichnet wird

1. **Roads (background layer)**
   - Polylines fuer alle Edges im sichtbaren Map-Bereich
   - Stroke: subtiler Grau-Ton (muted-foreground mit niedriger Opacity)
   - Stroke-Width: 1-2px je nach Zoom
   - Nur Edges im aktuellen Viewport rendern (Performance)

2. **Planned Route (foreground layer)**
   - Polyline durch die Route-Waypoints
   - Stroke: Brand-Akzent (Teal/Primary)
   - Stroke-Width: 3-4px
   - Gradient von Teal zu Cyan optional, sehr subtil
   - Nur wenn router-plugin Waypoints publisht

3. **Truck Position**
   - Truck-Icon (kleiner gefuellter Kreis oder Truck-SVG)
   - Brand-Akzent
   - Heading-Indicator: kleines Dreieck oder Linie in Heading-Richtung
   - Bei Engaged: leichter Pulse-Ring um den Truck

4. **Next Maneuver Marker**
   - Wenn naechster Waypoint < 500m: kleines Pin/Diamond am Maneuver-Punkt
   - Optional: Distance-Label daneben

5. **Goal Pin**
   - Wenn Goal gesetzt: Flag/Pin am Goal-Node
   - Brand-Purple-Akzent (vom Logo-Gradient)

### Debug-Layer (optional, toggleable)

- Alle Nodes als kleine Punkte (1-2px, muted)
- Edge-IDs an wichtigen Junctions
- Sektor-Grenzen als duenne Linien
- Hilft bei Routing-Debugging

## Interaktion

Minimal halten:

- **Pan**: Maus-Drag verschiebt die Karte. Bei Follow-Truck aktiv: Pan wird ueberschrieben.
- **Zoom**: Mausrad. 4-5 diskrete Zoom-Stufen, nicht stufenlos.
- **Click auf Node** (Debug-Mode): zeigt UID + Koordinaten als Tooltip.
- **Follow-Truck Toggle**: schaltet zwischen "Truck zentriert" und "frei navigierbar".

Keine Multi-Touch-Gesten, keine 3D-Effekte, kein Tilt.

## States

- **No Telemetry**: Karte ist sichtbar (statische Roads), Truck-Icon fehlt, Hint-Text unten "Waiting for telemetry"
- **No Route**: Truck + Roads sichtbar, keine Route-Polyline, Hint-Text "Set destination to see route"
- **Route Active**: alles sichtbar
- **Approaching Maneuver** (next < 200m): Marker am naechsten Maneuver-Punkt wird groesser/pulsed leicht
- **Off-Route**: Route-Polyline wird gedimmt (niedrigere Opacity), Hint-Text "Recalculating..."

# Technische Umsetzung

## SVG vs Canvas

**Empfehlung: SVG fuer den Anfang**, ggf. Canvas-Migration bei Performance-Problemen.

Begruendung:
- React + SVG ist deklarativ und passt zum Stack
- Bei sichtbarem Bereich von ~100 Roads ist SVG-Performance OK
- Canvas waere bei 1000+ sichtbaren Edges noetig
- Lucide-Icons sind schon SVG-basiert, Konsistenz

## Coordinate System

ETS2-Welt-Koordinaten sind in Metern. Bei 1.15M Nodes ueber Europa-Karte:
- Viewport-Bereich typisch: 500-5000m um den Truck
- Map-zu-Screen-Transformation: einfache Skalierung + Translation
- Y-Achse ETS2-Welt: nach unten -> bei SVG flippen (SVG y-Down)

## Data-Loading

graph.json ist gross (vermutlich 100MB+). NICHT komplett ins UI laden.

Optionen:
1. Spatial Index im Daemon: UI fragt "gib mir alle Nodes/Edges im Bbox X1,Z1,X2,Z2"
2. Daemon publisht Subset ins Blackboard (z.B. nur 5km um Truck)
3. UI laedt Sektoren als JSON-Chunks bei Bedarf

Fuer ersten MVP: Daemon publisht nearby_nodes und nearby_edges ins Blackboard, 1 Hz Update-Rate. UI rendert was kommt.

## Performance-Budget

- 60 FPS Target
- Pan/Zoom soll fluessig sein
- Maximal 200-500 Edges gleichzeitig im Viewport
- Truck-Position-Update via Telemetry (15-60 Hz) darf kein Re-Render der Karten-Background sein, nur des Truck-Icons

# Component-File

Datei: crates/ui/src/components/RouteMap.tsx

Props-Interface:

```tsx
interface RouteMapProps {
  // Live data
  truckPosition: { x: number; z: number } | null;
  truckHeading: number | null; // rad
  truckSpeed: number; // km/h
  
  // Map data (vom Daemon via Blackboard)
  nearbyEdges: MapEdge[];
  nearbyNodes: MapNode[];
  
  // Route data
  routeWaypoints: { x: number; z: number; uid: string }[];
  goalPosition: { x: number; z: number } | null;
  nextManeuverPosition: { x: number; z: number } | null;
  nextManeuverDistance: number | null; // meters
  
  // UI state
  isEngaged: boolean;
  isOffRoute: boolean;
  
  // Settings (per Card)
  followTruck?: boolean;
  showDebugLayer?: boolean;
}
```

Helper-Types in einem separaten File:

```tsx
// crates/ui/src/types/map.ts
interface MapNode {
  uid: string;
  position: { x: number; z: number };
  is_junction?: boolean;
}

interface MapEdge {
  from_uid: string;
  to_uid: string;
  from: { x: number; z: number };
  to: { x: number; z: number };
  road_type?: 'highway' | 'road' | 'prefab' | 'bezier';
}
```

# Daemon-Side: Was muss publisht werden

Im Blackboard unter map.* Prefix:

| Key | Type | Update Rate | Source |
|-----|------|-------------|--------|
| map.nearby_edges | MapEdge[] | 1 Hz | Daemon Spatial-Query |
| map.nearby_nodes | MapNode[] | 1 Hz | Daemon Spatial-Query |
| map.viewport_bbox | {x1,z1,x2,z2} | on truck-move | Daemon |
| route.waypoints | string[] (UIDs) | on plan/replan | router-plugin |
| route.goal_uid | string | on set-goal | router-plugin |
| route.next_maneuver_position | {x,z} | on maneuver-change | router-plugin |
| route.next_maneuver_distance | f32 | continuous | router-plugin |

# Visual Beispiel-Beschreibung

Dunkler Hintergrund (passt zum Dashboard).

Strassen als duenne graue Polylines, fast bis muted-foreground/30.

Eine kraeftige Teal-Linie schlaengelt sich durch das Netz - die geplante Route. Linie ist nicht uniform-dick, sondern hat einen leichten Gradient von einem Ende zum anderen, sehr subtil.

In der Mitte/leicht versetzt: kleiner Truck-Punkt in Teal, mit Heading-Pfeil. Leichter Pulse-Ring um den Truck wenn engaged.

Am Ende der Route: kleiner Pin in Purple (vom Logo-Gradient-End).

Bei Approaching-Maneuver: zusaetzlicher Marker am Maneuver-Punkt, leicht pulsierender Ring.

Footer: "Zoom 1:5000  •  Follow on  •  Debug off"

Im Debug-Mode kommen kleine Punkte fuer alle Nodes dazu, sehr klein und stark gedimmt (foreground/10), nur erkennbar wenn man weiss dass sie da sind.

# Was NICHT rein soll

- Google-Maps-Style mit Strassen-Beschriftung
- Country-Borders, Wasser, Gebirge
- Tile-basierte Karte
- 3D-Perspektive
- Animationen die ablenken (drehende Marker, etc.)
- Stadt-Namen ueberall (Goal-City reicht)
- Tageszeit-Effekte oder Wetter-Visualisierung

# Implementation-Reihenfolge

Phase 1: Statisches Skelett
- Component-Struktur, Props-Interface, SVG-Canvas
- Hardcoded Test-Daten (5 Nodes, 4 Edges, 1 Truck-Position)
- Zoom + Pan funktional
- Theme-konform

Phase 2: Live-Daten
- Blackboard-Subscription auf route.* und map.*
- Truck-Position-Update aus Telemetry-Stream
- Re-Render nur des Truck-Icons bei Position-Change

Phase 3: Daemon-Side
- Spatial-Query Funktion (basierend auf graph.json)
- map.nearby_edges Publish im Daemon
- Performance-Tuning

Phase 4: Route-Layer
- Route-Polyline rendern aus route.waypoints
- Goal-Pin
- Next-Maneuver-Marker

Phase 5: Polish
- Pulse-Animation am Truck bei Engaged
- Off-Route State
- Debug-Layer Toggle
- Follow-Truck Toggle Animation

# Output

Eine RouteMap.tsx als React+TS Component mit Props-Interface gemaess Spec oben. Plus Helper-Types in types/map.ts. Plus Anbindung-Hinweise im Implementation-Guide wie das Component im Dashboard eingebunden wird und welche Blackboard-Keys subscribed werden muessen.

Plus Daemon-Side Spec: welche neuen Blackboard-Keys gepublisht werden muessen und in welchem Plugin (vermutlich erweiterte router-plugin Logik oder ein neues map-publisher Plugin).

# Constraints

- React 19 + TypeScript 5.7 + Tailwind 4
- SVG-basiert, kein Canvas fuer MVP
- Tokens vom Design-System nutzen, keine Hardcoded-Farben
- prefers-reduced-motion: Pulse-Animations abschalten
- Performance: bei stillstehendem Truck darf das Component bei 0% CPU laufen (kein RAF-Spam)
- Beide Themes (Dark + Light) muessen funktionieren

# Start

Beginne mit Phase 1 (statisches Skelett mit Hardcoded-Daten). Beweise dass Pan/Zoom + Theme-konform funktioniert. Dann Phase 2.
</pasted_text>

## Assistant

_[tool: update_todos]_

_[tool: write_file]_

_[tool: write_file]_

Now the production RouteMap, replacing the earlier skeleton.

_[tool: write_file]_

_[tool: read_file]_

Let me also surface the new look in the visual spec so the user can see it without wiring up React.

_[tool: grep]_

_[tool: read_file]_

_[tool: str_replace_edit]_

_[tool: str_replace_edit]_

_[tool: str_replace_edit]_

_[tool: done]_

_[tool: fork_verifier_agent]_

Geliefert:

- **`design-system/types/map.ts`** — `MapNode`, `MapEdge`, `RouteWaypoint`, `WorldPoint`, `WorldBBox`. Wire-format identisch zu dem, was der Daemon publishen muss.
- **`design-system/components/RouteMap.tsx`** — vollständige ETS2LA-Style-Visualisierung mit dem geforderten Props-Interface. Highlights:
  - Single-Matrix-Transform auf dem äußeren `<g>` → Straßen-/Routen-`<path>`s memoizen auf `nearbyEdges`-Identität und re-rendern *nicht* bei Truck-Ticks. Nur die `transform`-Attribute bewegen sich.
  - 5 diskrete Zoom-Stufen (250 m – 5 km), Mausrad + +/−-Buttons, Pointer-Drag pant und deaktiviert Follow-Mode automatisch.
  - Roads gebucketet nach `road_type` zu einem `<path>` pro Bucket (deutlich weniger DOM-Knoten als per-Edge `<line>`).
  - Truck mit Heading-Pfeil + Pulse-Ring (nur bei `isEngaged`), Maneuver-Marker pulst bei `< 200 m`, Goal-Pin in Purple (vom Logo-Gradient-Ende), Off-Route dimmt die Route.
  - Empty-States als ruhige Pill-Hinweise, kein Banner.
  - `motion-reduce:[animation:none]` auf beiden Pulse-Stellen.
  - Idle: keine RAF/setInterval, 0 % CPU.
- **`design-system/route-map-integration.md`** — Dashboard-Wiring (Zustand-Selektoren mit `useShallow`-Hinweis), Blackboard-Key-Tabelle, Skizze für ein neues `map-publisher`-Plugin (R-Tree über `graph.json`), Erweiterung des `router-plugin` für die `route.*`-Keys, Performance-Mitigations.
- **`visual-spec.html`** — Route-Preview-Card in der Vorschau aktualisiert: Highway-Korridor + Nebenstraßen, Route in Teal, Truck mit Heading, pulsierender Maneuver-Marker, Goal-Pin in Purple, Live/Speed-Chips oben links, Zoom-Buttons oben rechts, Footer mit Follow/Debug-Toggles.

Caveats:
- Die Visual-Spec ist ein statischer Render; die echte Pan/Zoom-Interaktivität gibt es nur in `RouteMap.tsx` selbst (Component in eine Storybook-Story / Test-Route packen, um sie ohne Daemon zu probieren — siehe Phase 1 mit Hardcoded-Daten im Integration-Guide).
- `map-publisher` ist in dem Guide nur als Rust-Skizze; den R-Tree-Build aus `graph.json` musst du noch implementieren.
- Selektoren-Stabilität ist entscheidend — wenn `nearbyEdges` bei jedem Telemetry-Tick eine neue Array-Referenz liefert, verlierst du den `useMemo`-Vorteil.

## User

Verifier agent check completed

## Assistant

_[tool: str_replace_edit]_

Right — markers were drawn in world meters inside the scaled group, so they collapsed to a couple of pixels. Fix: project the marker positions to pixel space and render them outside the matrix transform with pixel radii. I'll patch the component and the visual spec demo.

_[tool: grep]_

_[tool: read_file]_

_[tool: str_replace_edit]_

_[tool: read_file]_

_[tool: str_replace_edit]_

_[tool: str_replace_edit]_

_[tool: done]_

_[tool: fork_verifier_agent]_

Markers are now drawn in pixel space (truck/maneuver/goal) outside the matrix group, with constant on-screen sizes regardless of zoom — both in the production `RouteMap.tsx` and the visual spec demo. Verifier is checking the rendered sizes.


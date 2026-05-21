# CAM & MCM — Diagramas de Mensagens ETSI V2X

## 1. Arquitectura de Comunicação V2X

Visão geral de como as mensagens CAM e MCM fluem entre os OBUs através do Vanetza e MQTT.

```mermaid
graph TB
    subgraph "Veículo A — OBU Container"
        APP_A["🧠 OBU Brain<br/>(Python main.py)"]
        VAT_A["📡 Vanetza Socktap<br/>(UPER Encode/Decode)"]
        MQTT_A["🔌 MQTT Broker Local"]
        
        APP_A -->|"Publica JSON"| MQTT_A
        MQTT_A -->|"vanetza/in/cam<br/>vanetza/in/mcm"| VAT_A
        VAT_A -->|"vanetza/out/cam<br/>vanetza/out/mcm"| MQTT_A
        MQTT_A -->|"Subscreve"| APP_A
    end

    subgraph "Rede V2X (GeoNetworking)"
        V2X["📶 Rede V2X<br/>(BTP / GeoNet)"]
    end

    subgraph "Veículo B — OBU Container"
        APP_B["🧠 OBU Brain<br/>(Python main.py)"]
        VAT_B["📡 Vanetza Socktap<br/>(UPER Encode/Decode)"]
        MQTT_B["🔌 MQTT Broker Local"]
        
        APP_B -->|"Publica JSON"| MQTT_B
        MQTT_B -->|"vanetza/in/cam<br/>vanetza/in/mcm"| VAT_B
        VAT_B -->|"vanetza/out/cam<br/>vanetza/out/mcm"| MQTT_B
        MQTT_B -->|"Subscreve"| APP_B
    end

    VAT_A <-->|"ETSI UPER<br/>(encoded PDU)"| V2X
    VAT_B <-->|"ETSI UPER<br/>(encoded PDU)"| V2X
```

---

## 2. CAM — Cooperative Awareness Message

### 2.1 Objetivo

A CAM é uma mensagem **periódica** enviada por cada veículo para anunciar a sua presença, posição, velocidade e estado cinemático. É o mecanismo de **awareness** — cada OBU constrói o seu mapa de tráfego local a partir das CAMs recebidas dos vizinhos.

### 2.2 Fluxo da CAM

```mermaid
sequenceDiagram
    participant S as Sensores SUMO
    participant OBU as OBU Brain
    participant LB as MQTT Local
    participant VAT as Vanetza
    participant V2X as Rede V2X
    participant VAT2 as Vanetza (Vizinho)
    participant OBU2 as OBU Brain (Vizinho)

    S->>OBU: car/<id>/sensors/gps (x, y, speed, heading)
    
    Note over OBU: A cada CAM_PERIOD_MS (100ms):<br/>_build_cam() converte XY→LatLon

    OBU->>LB: vanetza/in/cam (JSON)
    LB->>VAT: Recebe CAM JSON
    VAT->>V2X: Codifica UPER → Envia BTP/GeoNet
    V2X->>VAT2: PDU ETSI recebido
    VAT2->>OBU2: vanetza/out/cam (JSON descodificado)
    
    Note over OBU2: _handle_cam():<br/>Extrai posição, velocidade, heading<br/>Actualiza neighbors[stationId]
```

### 2.3 Estrutura da CAM

```mermaid
graph LR
    subgraph "CAM Payload (vanetza/in/cam)"
        CAM["camParameters"]
        BC["basicContainer"]
        HF["highFrequencyContainer"]
        LF["lowFrequencyContainer"]
        
        CAM --> BC
        CAM --> HF
        CAM --> LF
        
        BC --> RP["referencePosition<br/>• latitude<br/>• longitude<br/>• altitude"]
        BC --> ST["stationType: 5"]
        
        HF --> BV["basicVehicleContainer<br/>HighFrequency"]
        BV --> SPD["speed<br/>• speedValue (m/s)<br/>• speedConfidence"]
        BV --> HDG["heading<br/>• headingValue (°)<br/>• headingConfidence"]
        BV --> VL["vehicleLength / Width"]
        BV --> ACC["longitudinalAcceleration"]
        
        LF --> VR["vehicleRole: 0"]
        LF --> EL["exteriorLights"]
    end
```

### 2.4 Campos-Chave Utilizados pelo OBU

| Campo | Caminho JSON | Utilização |
|-------|-------------|------------|
| **Station ID** | `fields.header.stationId` | Identificação única do veículo |
| **Latitude** | `fields.cam.camParameters.basicContainer.referencePosition.latitude` | Conversão LatLon → XY SUMO |
| **Longitude** | `fields.cam.camParameters.basicContainer.referencePosition.longitude` | Conversão LatLon → XY SUMO |
| **Speed** | `fields.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.speed.speedValue` | Cálculo de ETA e car-following |
| **Heading** | `fields.cam.camParameters.highFrequencyContainer.basicVehicleContainerHighFrequency.heading.headingValue` | Projeção de trajetória e TTC |

### 2.5 Utilização da CAM no Sistema

```mermaid
graph TD
    CAM_RX["CAM Recebida<br/>(_handle_cam)"] --> CONV["Converte LatLon → XY<br/>(latlon_to_xy)"]
    CONV --> UPD["Actualiza neighbors[stationId]<br/>x, y, speed, heading,<br/>distance_to_merge, timestamp"]
    UPD --> ROLE["Detecção de Role<br/>(merge / host / lead)"]
    UPD --> ETA["Cálculo de ETA<br/>distance / speed"]
    UPD --> FOL["CAM-Following<br/>(car-following cooperativo)"]
    UPD --> FG["Final Guard<br/>(TTC dinâmico)"]
    
    ROLE --> FSM["Máquina de Estados FSM"]
    ETA --> FSM
```

---

## 3. MCM — Maneuver Coordination Message

### 3.1 Objetivo

A MCM é uma mensagem de **coordenação de manobra** utilizada para negociar a fusão (merge) entre veículos. Implementa um protocolo REQUEST → ACCEPT / REJECT que permite ao veículo da rampa pedir autorização ao host (veículo da via principal) para entrar.

### 3.2 Tipos de Ação MCM

| Código | Nome | Descrição |
|--------|------|-----------|
| `1` | **REQUEST** | Veículo merge pede autorização para entrar |
| `2` | **ACCEPT** | Veículo host autoriza a manobra |
| `3` | **REJECT** | Veículo host recusa (demasiado perto do merge point) |

> [!IMPORTANT]
> O campo `manoeuvreCooperationCost` no `basicContainer.rational` é utilizado como o campo de ação (1=REQUEST, 2=ACCEPT, 3=REJECT). O `manoeuvreId` deve estar no intervalo `0..255` (Identifier1B).

### 3.3 Fluxo de Negociação MCM

```mermaid
sequenceDiagram
    participant MV as 🚗 Merge Vehicle<br/>(Rampa)
    participant V2X as 📶 Rede V2X
    participant HV as 🚙 Host Vehicle<br/>(Via Principal)

    Note over MV: Estado: CRUISE → NEGOTIATING<br/>Detecta host via CAMs<br/>DTM < MCM_REQUEST_DISTANCE (95m)

    MV->>V2X: MCM REQUEST<br/>(action=1, manoeuvreId=N,<br/>target=host_station_id)
    V2X->>HV: MCM REQUEST recebido

    Note over HV: _handle_mcm():<br/>Verifica se é o target<br/>Verifica distância ao merge

    alt Distância > HOST_REJECT_DISTANCE (20m)
        Note over HV: Estado: CRUISE → YIELDING<br/>Reduz velocidade para abrir gap
        HV->>V2X: MCM ACCEPT<br/>(action=2, manoeuvreId=N,<br/>target=merge_station_id)
        V2X->>MV: MCM ACCEPT recebido
        
        Note over MV: MCM_ACCEPT_MATCHED!<br/>MERGE_AUTHORIZED_BY_MCM<br/>Estado: NEGOTIATING → MERGING
        
        MV->>MV: Executa mudança de faixa<br/>target_lane = merge_lane_index
        
        Note over MV: Após completar:<br/>MERGE_COMPLETED<br/>Estado → CRUISE
    else Distância ≤ HOST_REJECT_DISTANCE (20m)
        HV->>V2X: MCM REJECT<br/>(action=3, manoeuvreId=N)
        V2X->>MV: MCM REJECT recebido
        
        Note over MV: Estado → ABORT<br/>Cooldown antes de retry
    end
```

### 3.4 Estrutura da MCM

```mermaid
graph LR
    subgraph "MCM Payload (vanetza/in/mcm)"
        BC2["basicContainer"]
        MC["mcmContainer"]
        
        BC2 --> GDT["generationDeltaTime"]
        BC2 --> SID["stationID"]
        BC2 --> MCT["mcmType: 8"]
        BC2 --> MID["manoeuvreId: 0..255"]
        BC2 --> RAT["rational<br/>• manoeuvreCooperationCost<br/>(1=REQ, 2=ACC, 3=REJ)"]
        BC2 --> POS["position<br/>• latitude<br/>• longitude"]
        
        MC --> VMC["vehicleManoeuvreContainer"]
        VMC --> VCS["vehicleCurrentStateContainer<br/>• vehicleSpeed<br/>• vehicleHeading<br/>• vehicleSize"]
        VMC --> SUB["submaneuvres<br/>• referenceTrajectory<br/>• targetRoadResource"]
        VMC --> ADV["manoeuvreAdvice<br/>• executantID (target)<br/>• advisedTrajectory"]
    end
```

### 3.5 Campos-Chave Utilizados pelo OBU

| Campo | Caminho JSON | Utilização |
|-------|-------------|------------|
| **Station ID** | `fields.header.stationId` ou `basicContainer.stationID` | Quem enviou a MCM |
| **Action** | `basicContainer.rational.manoeuvreCooperationCost` | Tipo: REQUEST(1) / ACCEPT(2) / REJECT(3) |
| **Manoeuvre ID** | `basicContainer.manoeuvreId` | Identificador da manobra (0..255) |
| **Target** | `mcmContainer.vehicleManoeuvreContainer.manoeuvreAdvice[0].executantID` | Para quem se destina |
| **Speed** | `mcmContainer.vehicleManoeuvreContainer.vehicleCurrentStateContainer.vehicleSpeed.speedValue` | Velocidade actual do emissor |
| **Position** | `basicContainer.position.latitude / longitude` | Posição actual do emissor |

---

## 4. Integração CAM + MCM no FSM

```mermaid
stateDiagram-v2
    [*] --> CRUISE

    CRUISE --> NEGOTIATING: Merge vehicle detecta host<br/>& DTM < 95m<br/>Envia MCM REQUEST
    
    CRUISE --> YIELDING: Host recebe MCM REQUEST<br/>& distância segura<br/>Envia MCM ACCEPT

    NEGOTIATING --> MERGING: MCM ACCEPT recebido<br/>& gap seguro<br/>MERGE_AUTHORIZED_BY_MCM
    
    NEGOTIATING --> ABORT: MCM REJECT recebido<br/>ou timeout (2s)
    
    YIELDING --> CRUISE: Merge concluído<br/>ou timeout
    
    MERGING --> CRUISE: MERGE_COMPLETED<br/>(na faixa principal)
    
    ABORT --> CRUISE: Após cooldown (3s)
    
    CRUISE --> CRUISE: CAMs periódicas<br/>actualizam neighbors

    note right of CRUISE
        CAMs são enviadas periodicamente
        por TODOS os veículos em TODOS os estados.
        O mapa de tráfego é sempre actualizado.
    end note
    
    note right of NEGOTIATING
        MCM REQUESTs são re-enviadas
        a cada REQUEST_RETRY_S (0.5s)
        até timeout ou resposta.
    end note
    
    note left of YIELDING
        Host reduz velocidade
        e envia MCM ACCEPT
        periodicamente (RESPONSE_PERIOD_S)
    end note
```

---

## 5. Tópicos MQTT — Resumo

```mermaid
graph LR
    subgraph "OBU App → Vanetza (Encoding)"
        IN_CAM["vanetza/in/cam"]
        IN_MCM["vanetza/in/mcm"]
        IN_DENM["vanetza/in/denm"]
    end
    
    subgraph "Vanetza → OBU App (Decoding)"
        OUT_CAM["vanetza/out/cam"]
        OUT_MCM["vanetza/out/mcm"]
        OUT_DENM["vanetza/out/denm"]
    end
    
    subgraph "Envelope Vanetza-NAP"
        ENV["Decoded payload under 'fields'"]
        ENV --> CAM_F["CAM: fields.header + fields.cam"]
        ENV --> MCM_F["MCM: fields.header + fields.payload"]
        ENV --> DENM_F["DENM: fields.header + fields.denm"]
    end
```

---

## 6. Comparação CAM vs MCM

| Aspecto | CAM | MCM |
|---------|-----|-----|
| **Tipo** | Periódica (awareness) | Orientada a eventos (negociação) |
| **Frequência** | Cada `CAM_PERIOD_MS` (100ms) | Quando necessário (REQUEST/ACCEPT/REJECT) |
| **Direção** | Broadcast (todos os vizinhos) | Dirigida (via `executantID`) |
| **Conteúdo Principal** | Posição, velocidade, heading | Ação de coordenação, manoeuvreId, target |
| **Standard ETSI** | EN 302 637-2 | ETSI TS 103 561 |
| **Envelope Vanetza** | `fields.cam` | `fields.payload` |
| **Função no Sistema** | Mapa de tráfego, ETA, car-following | Negociação de merge (REQ/ACC/REJ) |
| **Todos os estados?** | ✅ Sempre enviada | ❌ Só em NEGOTIATING/YIELDING |

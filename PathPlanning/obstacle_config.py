class ObstacleConfig:
    """
    Classe centralizzata per tutti i parametri di rilevazione, clustering,
    tracking e sicurezza degli ostacoli.
    Modifica i valori qui sotto per ottimizzare il comportamento del LiDAR e del PathPlanner.
    """

    # =========================================================================
    # 1. PARAMETRI NUVOLA DI PUNTI E CAMPIONAMENTO (Pre-processing)
    # =========================================================================

    # Numero massimo di punti LiDAR mantenuti in memoria nella point cloud accumulata.
    # - Aumentalo: avrai ostacoli più definiti nel tempo, ma maggior carico computazionale e "scie" se gli oggetti si muovono.
    # - Diminuiscilo: reazione più rapida ai cambiamenti dinamici, ma nuvola più rada.
    _MAX_ACCUMULATED_POINTS = 30000

    # Intervallo di tempo (in secondi) tra un'esecuzione di DBSCAN e la successiva.
    # - Aumentalo: alleggerisce la CPU, ma aumenta la latenza nel rilevare nuovi ostacoli.
    # - Diminuiscilo: rilevazione più reattiva, ma maggior consumo di CPU.
    _CLUSTERING_INTERVAL_SEC = 0.3

    # Numero minimo di punti totali nella nuvola per far partire l'algoritmo di rilevazione.
    # Evita di lanciare DBSCAN quando la nuvola è ancora quasi vuota all'avvio.
    _MIN_TOTAL_POINTS_TO_PROCESS = 50

    # Dimensione (in metri) della cella (voxel) usata per ridurre la densità dei punti (downsampling).
    # - Aumentalo: riduce drasticamente i punti velocizzando DBSCAN, ma perde dettagli sugli ostacoli piccoli.
    # - Diminuiscilo: mantiene più punti e precisione geometrica, ma rallenta il clustering (richiede di alzare MIN_POINTS).
    _VOXEL_SIZE = 0.5


    # =========================================================================
    # 2. PARAMETRI DI CLUSTERING (Algoritmo DBSCAN) E BOUNDING BOX
    # =========================================================================

    # Raggio massimo (in metri) entro cui DBSCAN cerca punti vicini per unirli nello stesso ostacolo.
    # - Aumentalo: unisce punti più distanti (utile se il LiDAR è rado), ma rischia di fondere due ostacoli vicini in uno solo.
    # - Diminuiscilo: separa meglio gli ostacoli vicini, ma rischia di spezzare un singolo ostacolo grande in più pezzi.
    _DBSCAN_EPS = 4.5

    # Numero minimo di punti (dopo il voxel downsampling) entro il raggio EPS per formare un ostacolo valido.
    # - Aumentalo: elimina i falsi positivi e il rumore, ma potrebbe ignorare ostacoli piccoli o lontani (colpiti da pochi raggi).
    # - Diminuiscilo: rileva anche ostacoli piccoli o lontani, ma aumenta il rischio di rilevare "ostacoli fantasma" dovuti al rumore.
    _DBSCAN_MIN_POINTS = 45

    # Fattore di espansione percentuale (es. 0.1 = +10%) applicato a larghezza (X) e profondità (Y) del cluster.
    # Serve a compensare il fatto che il LiDAR vede solo la "faccia" anteriore dell'ostacolo.
    _CLUSTER_XY_EXPANSION_FACTOR = 0.1


    # =========================================================================
    # 3. PARAMETRI DI TRACKING E PROMOZIONE A OSTACOLO STATICO
    # =========================================================================

    # Distanza massima (in metri) tra il centro di un nuovo cluster e uno già in memoria
    # per considerarli lo stesso oggetto tra un frame e l'altro.
    # - Aumentalo: riesce a seguire ostacoli che si muovono molto velocemente.
    # - Diminuiscilo: evita di confondere due ostacoli diversi vicini tra loro.
    _TRACKING_MAX_MATCH_DISTANCE = 4.0

    # Soglia di movimento (in metri): se il centro del cluster si sposta di oltre questo valore
    # rispetto all'ultima posizione stabile, l'ostacolo viene classificato come "in movimento"
    # e il suo timer di stazionarietà viene azzerato.
    # - Aumentalo: tollera maggiormente le oscillazioni del bounding box (quando il LiDAR scopre nuovi lati dell'oggetto).
    # - Diminuiscilo: richiede che il centro del cluster sia quasi immobile prima di promuoverlo a statico.
    _MOVEMENT_THRESHOLD_DISTANCE = 2.5

    # Tempo minimo (in secondi) in cui un nuovo ostacolo deve rimanere fermo (sotto la soglia di movimento)
    # prima di essere "promosso" a ostacolo statico definitivo e inviato al PathManager.
    # - Aumentalo: evita di aggiungere alla mappa ostacoli mobili o letture instabili, ma ritarda la frenata/ricalcolo.
    # - Diminuiscilo: il drone reagisce e ricalcola la rotta quasi istantaneamente, ma rischia di mappare oggetti solo rallentati.
    _STATIC_PROMOTION_TIME_SEC = 0.9

    # Tempo di oblio (in secondi): se un ostacolo tracciato (non ancora promosso a statico)
    # non viene più visto dal LiDAR per questo intervallo di tempo, viene eliminato dalla memoria.
    _OBSTACLE_FORGET_TIME_SEC = 2.0


    # =========================================================================
    # 4. PARAMETRI DI SICUREZZA E ANTI-COLLISIONE (Path Planning)
    # =========================================================================

    # Raggio di sicurezza del drone (in metri) aggiunto attorno ai nuovi ostacoli statici rilevati.
    # Espande il bounding box (AABB) in tutte le direzioni per garantire che il drone passi a distanza di sicurezza.
    _DRONE_SAFETY_RADIUS = 7.0

    # Raggio di sicurezza di default (in metri) usato dalla classe ObstacleAABB se non specificato diversamente.
    _DEFAULT_AABB_DRONE_RADIUS = 5.0

    # Risoluzione dei controlli lungo i segmenti della rotta (1 controllo ogni X metri) e numero minimo di step.
    _SEGMENT_CHECK_STEP_METERS = 1.0
    _SEGMENT_CHECK_MIN_STEPS = 5

    # Distanza di retromarcia di sicurezza (in metri) eseguita prima del ricalcolo della rotta in caso di emergenza.
    _EMERGENCY_BACKUP_DISTANCE = 4.0


    # =========================================================================
    # METODI GETTER
    # =========================================================================

    @classmethod
    def get_max_accumulated_points(cls) -> int:
        return cls._MAX_ACCUMULATED_POINTS

    @classmethod
    def get_clustering_interval_sec(cls) -> float:
        return cls._CLUSTERING_INTERVAL_SEC

    @classmethod
    def get_min_total_points_to_process(cls) -> int:
        return cls._MIN_TOTAL_POINTS_TO_PROCESS

    @classmethod
    def get_voxel_size(cls) -> float:
        return cls._VOXEL_SIZE

    @classmethod
    def get_dbscan_eps(cls) -> float:
        return cls._DBSCAN_EPS

    @classmethod
    def get_dbscan_min_points(cls) -> int:
        return cls._DBSCAN_MIN_POINTS

    @classmethod
    def get_cluster_xy_expansion_factor(cls) -> float:
        return cls._CLUSTER_XY_EXPANSION_FACTOR

    @classmethod
    def get_tracking_max_match_distance(cls) -> float:
        return cls._TRACKING_MAX_MATCH_DISTANCE

    @classmethod
    def get_movement_threshold_distance(cls) -> float:
        return cls._MOVEMENT_THRESHOLD_DISTANCE

    @classmethod
    def get_static_promotion_time_sec(cls) -> float:
        return cls._STATIC_PROMOTION_TIME_SEC

    @classmethod
    def get_obstacle_forget_time_sec(cls) -> float:
        return cls._OBSTACLE_FORGET_TIME_SEC

    @classmethod
    def get_drone_safety_radius(cls) -> float:
        return cls._DRONE_SAFETY_RADIUS

    @classmethod
    def get_default_aabb_drone_radius(cls) -> float:
        return cls._DEFAULT_AABB_DRONE_RADIUS

    @classmethod
    def get_segment_check_step_meters(cls) -> float:
        return cls._SEGMENT_CHECK_STEP_METERS

    @classmethod
    def get_segment_check_min_steps(cls) -> int:
        return cls._SEGMENT_CHECK_MIN_STEPS

    @classmethod
    def get_emergency_backup_distance(cls) -> float:
        return cls._EMERGENCY_BACKUP_DISTANCE
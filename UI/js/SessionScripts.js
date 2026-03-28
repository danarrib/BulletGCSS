// SessionScripts.js
// IndexedDB persistence layer for flight sessions.
// Level 0 — no imports from other app modules.
//
// Schema:
//   sessions store:         { id (autoIncrement), name, status ('open'|'closed'), startTime, lastUpdate }
//   session_messages store: { id (autoIncrement), sessionId, line }  where line = "timestamp|payload"

const DB_NAME = 'BulletGCSS';
const DB_VERSION = 1;

let db = null;

export function openDB() {
    return new Promise(function(resolve, reject) {
        var request = indexedDB.open(DB_NAME, DB_VERSION);

        request.onupgradeneeded = function(event) {
            var database = event.target.result;

            if (!database.objectStoreNames.contains('sessions')) {
                var sessionsStore = database.createObjectStore('sessions', { keyPath: 'id', autoIncrement: true });
                sessionsStore.createIndex('status', 'status', { unique: false });
            }

            if (!database.objectStoreNames.contains('session_messages')) {
                var messagesStore = database.createObjectStore('session_messages', { keyPath: 'id', autoIncrement: true });
                messagesStore.createIndex('sessionId', 'sessionId', { unique: false });
            }
        };

        request.onsuccess = function(event) {
            db = event.target.result;
            resolve(db);
        };

        request.onerror = function(event) {
            reject(event.target.error);
        };
    });
}

export function createSession(name) {
    return new Promise(function(resolve, reject) {
        var tx = db.transaction('sessions', 'readwrite');
        var store = tx.objectStore('sessions');
        var session = {
            name: name,
            status: 'open',
            startTime: Date.now(),
            lastUpdate: Date.now(),
        };
        var request = store.add(session);
        request.onsuccess = function() { resolve(request.result); };
        request.onerror = function() { reject(request.error); };
    });
}

export function closeSession(id) {
    return new Promise(function(resolve, reject) {
        var tx = db.transaction('sessions', 'readwrite');
        var store = tx.objectStore('sessions');
        var getReq = store.get(id);
        getReq.onsuccess = function() {
            var session = getReq.result;
            if (!session) { resolve(); return; }
            session.status = 'closed';
            session.lastUpdate = Date.now();
            var putReq = store.put(session);
            putReq.onsuccess = function() { resolve(); };
            putReq.onerror = function() { reject(putReq.error); };
        };
        getReq.onerror = function() { reject(getReq.error); };
    });
}

// Fire-and-forget: appends a "timestamp|payload" line to a session.
// Called on every MQTT message — any error is silently logged.
export function appendMessage(sessionId, line) {
    if (!db) return;
    try {
        var tx = db.transaction('session_messages', 'readwrite');
        tx.objectStore('session_messages').add({ sessionId: sessionId, line: line });
    } catch (e) {
        console.error('appendMessage error:', e);
    }
}

export function getOpenSession() {
    return new Promise(function(resolve, reject) {
        var tx = db.transaction('sessions', 'readonly');
        var index = tx.objectStore('sessions').index('status');
        var request = index.get('open');
        request.onsuccess = function() { resolve(request.result || null); };
        request.onerror = function() { reject(request.error); };
    });
}

export function listSessions() {
    return new Promise(function(resolve, reject) {
        var tx = db.transaction('sessions', 'readonly');
        var request = tx.objectStore('sessions').getAll();
        request.onsuccess = function() {
            var sessions = request.result.sort(function(a, b) { return b.startTime - a.startTime; });
            resolve(sessions);
        };
        request.onerror = function() { reject(request.error); };
    });
}

export function countSessionMessages(sessionId) {
    return new Promise(function(resolve, reject) {
        var tx = db.transaction('session_messages', 'readonly');
        var index = tx.objectStore('session_messages').index('sessionId');
        var request = index.count(IDBKeyRange.only(sessionId));
        request.onsuccess = function() { resolve(request.result); };
        request.onerror = function() { reject(request.error); };
    });
}

export function getSessionMessages(sessionId) {
    return new Promise(function(resolve, reject) {
        var tx = db.transaction('session_messages', 'readonly');
        var index = tx.objectStore('session_messages').index('sessionId');
        var request = index.getAll(sessionId);
        request.onsuccess = function() { resolve(request.result); };
        request.onerror = function() { reject(request.error); };
    });
}

export function deleteSession(sessionId) {
    return new Promise(function(resolve, reject) {
        var tx = db.transaction(['sessions', 'session_messages'], 'readwrite');

        tx.objectStore('sessions').delete(sessionId);

        var index = tx.objectStore('session_messages').index('sessionId');
        var cursorReq = index.openCursor(IDBKeyRange.only(sessionId));
        cursorReq.onsuccess = function(event) {
            var cursor = event.target.result;
            if (cursor) {
                cursor.delete();
                cursor.continue();
            }
        };

        tx.oncomplete = function() { resolve(); };
        tx.onerror = function() { reject(tx.error); };
    });
}

export function renameSession(sessionId, name) {
    return new Promise(function(resolve, reject) {
        var tx = db.transaction('sessions', 'readwrite');
        var store = tx.objectStore('sessions');
        var getReq = store.get(sessionId);
        getReq.onsuccess = function() {
            var session = getReq.result;
            if (!session) { resolve(); return; }
            session.name = name;
            var putReq = store.put(session);
            putReq.onsuccess = function() { resolve(); };
            putReq.onerror = function() { reject(putReq.error); };
        };
        getReq.onerror = function() { reject(getReq.error); };
    });
}

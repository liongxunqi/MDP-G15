package com.example.mdp

import java.io.IOException
import java.util.concurrent.CountDownLatch
import java.util.concurrent.TimeUnit
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.awaitCancellation
import kotlinx.coroutines.runBlocking
import kotlinx.coroutines.withContext
import org.junit.Assert.*
import org.junit.Test

class BluetoothSessionTest {
    @Test(timeout = 5000)
    fun `write failure closes transport and releases a blocked reader`() = runBlocking {
        val reading = CountDownLatch(1)
        val closed = CountDownLatch(1)
        try {
            withContext(Dispatchers.IO) {
                runBluetoothSession(
                    close = { closed.countDown() },
                    read = {
                        reading.countDown()
                        assertTrue("Read was never unblocked", closed.await(2, TimeUnit.SECONDS))
                        throw IOException("Read released by socket close")
                    },
                    write = {
                        assertTrue(reading.await(2, TimeUnit.SECONDS))
                        throw IOException("Simulated broken connection")
                    },
                    heartbeat = { awaitCancellation() },
                )
            }
            fail("Session must fail so the connection loop can retry")
        } catch (_: IOException) {
            assertEquals(0L, closed.count)
        }
    }

    @Test(timeout = 5000)
    fun `reader failure stops writer and heartbeat and closes transport`() = runBlocking {
        val closed = CountDownLatch(1)
        try {
            runBluetoothSession(
                close = { closed.countDown() },
                read = { throw IOException("Disconnected") },
                write = { awaitCancellation() },
                heartbeat = { awaitCancellation() },
            )
            fail("Expected session failure")
        } catch (_: IOException) {
            assertEquals(0L, closed.count)
        }
    }
}

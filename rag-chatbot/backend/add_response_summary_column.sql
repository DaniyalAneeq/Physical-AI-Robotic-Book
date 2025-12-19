-- Add response_summary column to query_logs table if it doesn't exist
DO $$
BEGIN
    IF NOT EXISTS (
        SELECT 1
        FROM information_schema.columns
        WHERE table_name = 'query_logs'
        AND column_name = 'response_summary'
    ) THEN
        ALTER TABLE query_logs
        ADD COLUMN response_summary TEXT;
        
        COMMENT ON COLUMN query_logs.response_summary IS 'First 200 chars of response';
    END IF;
END $$;
